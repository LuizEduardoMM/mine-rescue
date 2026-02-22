"""
Ambiente da mina subterrânea.
Grafo multinível com túneis de custo variável e colapsos progressivos.
Herda de agents.Environment (aima-python).
"""

import sys
import os
import copy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'aima-python'))

from agents import Environment, Agent, Thing


class Miner(Thing):
    """Minerador preso. priority=1 alta, oxygen=passos até falecer."""

    def __init__(self, priority=1, oxygen=20):
        self.priority = priority
        self.oxygen = oxygen
        self.alive = True
        self.rescued = False

    def __repr__(self):
        status = "VIVO" if self.alive else "FALECIDO"
        return f"<Minerador P{self.priority} O2={self.oxygen} {status}>"

    def tick(self):
        if self.alive and not self.rescued:
            self.oxygen -= 1
            if self.oxygen <= 0:
                self.alive = False


class Debris(Thing):
    def __repr__(self):
        return "<Escombros>"


class RescueRobot(Agent):

    def __init__(self, program=None, battery=50):
        super().__init__(program)
        self.battery = battery
        self.rescued_miners = []
        self.alive = True

    def __repr__(self):
        return f"<RobôResgate bat={self.battery} resgatados={len(self.rescued_miners)}>"


TUNNEL_COSTS = {
    'normal': 1,
    'estreito': 2,
    'inundado': 3,
    'instavel': 4,
    'elevador': 2,
}


class Tunnel:
    def __init__(self, from_node, to_node, tunnel_type='normal'):
        self.from_node = from_node
        self.to_node = to_node
        self.tunnel_type = tunnel_type
        self.cost = TUNNEL_COSTS.get(tunnel_type, 1)
        self.collapsed = False

    def __repr__(self):
        status = " [COLAPSADO]" if self.collapsed else ""
        return f"<Túnel {self.from_node}->{self.to_node} ({self.tunnel_type}, custo={self.cost}){status}>"


class MineEnvironment(Environment):
    """Ambiente da mina. Gerencia grafo, colapsos, percepções e ações."""

    def __init__(self, mine_graph, collapse_schedule=None):
        super().__init__()
        self.step_count = 0
        self.max_steps = 100

        self.tunnels = {}
        self.adjacency = {}

        for node, neighbors in mine_graph.items():
            if node not in self.adjacency:
                self.adjacency[node] = []
            for neighbor, tunnel_type in neighbors:
                if (node, neighbor) not in self.tunnels:
                    tunnel = Tunnel(node, neighbor, tunnel_type)
                    self.tunnels[(node, neighbor)] = tunnel
                if neighbor not in self.adjacency.get(node, []):
                    self.adjacency[node].append(neighbor)
                if (neighbor, node) not in self.tunnels:
                    reverse_tunnel = Tunnel(neighbor, node, tunnel_type)
                    self.tunnels[(neighbor, node)] = reverse_tunnel
                if neighbor not in self.adjacency:
                    self.adjacency[neighbor] = []
                if node not in self.adjacency[neighbor]:
                    self.adjacency[neighbor].append(node)

        self.collapse_schedule = {}
        if collapse_schedule:
            for from_node, to_node, collapse_step in collapse_schedule:
                self.collapse_schedule[(from_node, to_node)] = collapse_step
                self.collapse_schedule[(to_node, from_node)] = collapse_step

        self.history = []

    def percept(self, agent):
        """Percepção: posição, túneis adjacentes, mineradores aqui, bateria, step."""
        location = agent.location

        adjacent = []
        for neighbor in self.adjacency.get(location, []):
            tunnel = self.tunnels.get((location, neighbor))
            if tunnel and not tunnel.collapsed:
                adjacent.append({
                    'destination': neighbor,
                    'type': tunnel.tunnel_type,
                    'cost': tunnel.cost
                })

        miners_here = [thing for thing in self.list_things_at(location, Miner)
                       if thing.alive and not thing.rescued]

        return {
            'location': location,
            'adjacent_tunnels': adjacent,
            'miners_here': miners_here,
            'battery': agent.battery,
            'step': self.step_count,
            'rescued_miners': list(agent.rescued_miners),
            'all_miners': self._get_all_active_miners(),
        }

    def _get_all_active_miners(self):
        active = []
        for thing in self.things:
            if isinstance(thing, Miner) and thing.alive and not thing.rescued:
                active.append({
                    'location': thing.location,
                    'priority': thing.priority,
                    'oxygen': thing.oxygen,
                })
        return active

    def execute_action(self, agent, action):
        if action is None:
            return

        if not isinstance(agent, RescueRobot):
            return

        if agent.battery <= 0:
            agent.alive = False
            return

        if isinstance(action, tuple):
            action_type = action[0]

            if action_type == 'move':
                destination = action[1]
                self._execute_move(agent, destination)

            elif action_type == 'rescue':
                self._execute_rescue(agent)

        elif action == 'wait':
            agent.battery -= 1

    def _execute_move(self, agent, destination):
        location = agent.location
        tunnel = self.tunnels.get((location, destination))

        if tunnel is None:
            return  # Túnel não existe

        if tunnel.collapsed:
            return  # Túnel colapsado

        if destination not in self.adjacency.get(location, []):
            return  # Não é adjacente

        cost = tunnel.cost
        if agent.battery < cost:
            agent.alive = False
            return  # Sem bateria

        agent.battery -= cost
        agent.location = destination
        agent.performance -= cost  # Penalidade por custo de movimento

    def _execute_rescue(self, agent):
        location = agent.location
        miners = [t for t in self.list_things_at(location, Miner)
                  if t.alive and not t.rescued]

        for miner in miners:
            miner.rescued = True
            agent.rescued_miners.append(miner)
            reward = {1: 100, 2: 50, 3: 25}.get(miner.priority, 25)
            agent.performance += reward
            agent.battery -= 1  # Custo de resgate

    def exogenous_change(self):
        """Colapsos e redução de O2 a cada passo."""
        self.step_count += 1

        for (from_node, to_node), collapse_step in self.collapse_schedule.items():
            if self.step_count >= collapse_step:
                tunnel = self.tunnels.get((from_node, to_node))
                if tunnel and not tunnel.collapsed:
                    tunnel.collapsed = True

        for thing in self.things:
            if isinstance(thing, Miner):
                thing.tick()
                if not thing.alive and not thing.rescued:
                    for ag in self.agents:
                        penalty = {1: -200, 2: -100, 3: -50}.get(thing.priority, -50)
                        ag.performance += penalty

    def is_done(self):
        if self.step_count >= self.max_steps:
            return True

        if not any(agent.alive for agent in self.agents):
            return True

        active_miners = [t for t in self.things
                         if isinstance(t, Miner) and t.alive and not t.rescued]
        return len(active_miners) == 0

    def get_available_neighbors(self, location):
        neighbors = []
        for neighbor in self.adjacency.get(location, []):
            tunnel = self.tunnels.get((location, neighbor))
            if tunnel and not tunnel.collapsed:
                neighbors.append((neighbor, tunnel.cost, tunnel.tunnel_type))
        return neighbors

    def get_tunnel_cost(self, from_node, to_node):
        tunnel = self.tunnels.get((from_node, to_node))
        if tunnel and not tunnel.collapsed:
            return tunnel.cost
        return float('inf')

    def get_all_nodes(self):
        return list(self.adjacency.keys())

    def get_miner_locations(self):
        locations = []
        for thing in self.things:
            if isinstance(thing, Miner) and thing.alive and not thing.rescued:
                locations.append((thing.location, thing.priority, thing.oxygen))
        return locations

    def render(self):
        print("=" * 60)
        print(f"  MINA SUBTERRÂNEA - Passo {self.step_count}")
        print("=" * 60)

        levels = {}
        for node in self.adjacency:
            level = node[0]
            if level not in levels:
                levels[level] = []
            levels[level].append(node)

        for level in sorted(levels.keys()):
            nodes = sorted(levels[level], key=lambda x: x[1])
            print(f"\n  --- Nível {level} (profundidade) ---")

            for node in nodes:
                parts = [f"  [{node[1]}]"]

                for agent in self.agents:
                    if agent.location == node:
                        parts.append("[R]")

                miners = [t for t in self.list_things_at(node, Miner)]
                for m in miners:
                    if m.rescued:
                        parts.append("[OK]")
                    elif m.alive:
                        parts.append(f"[V]P{m.priority}(O2={m.oxygen})")
                    else:
                        parts.append("[X]")

                connections = []
                for neighbor in self.adjacency.get(node, []):
                    tunnel = self.tunnels.get((node, neighbor))
                    if tunnel:
                        status = "[X]" if tunnel.collapsed else "-"
                        connections.append(
                            f"{neighbor[1]}({tunnel.tunnel_type[0]}{status})")

                if connections:
                    parts.append(f"  -> {', '.join(connections)}")

                print(" ".join(parts))

        print(f"\n  --- Status do Agente ---")
        for agent in self.agents:
            if isinstance(agent, RescueRobot):
                print(f"  Posição: {agent.location}")
                print(f"  Bateria: {agent.battery}")
                print(f"  Resgatados: {len(agent.rescued_miners)}")
                print(f"  Performance: {agent.performance}")
                print(f"  Vivo: {agent.alive}")

        active = [t for t in self.things
                  if isinstance(t, Miner) and t.alive and not t.rescued]
        rescued = [t for t in self.things
                   if isinstance(t, Miner) and t.rescued]
        dead = [t for t in self.things
                if isinstance(t, Miner) and not t.alive and not t.rescued]

        print(f"\n  Mineradores: {len(active)} pendentes | "
              f"{len(rescued)} resgatados | {len(dead)} falecidos")
        print("=" * 60)


def create_default_mine():
    """
    Cria um cenário padrão de mina com 3 níveis.

    Nível 0 (superfície):
        Entrada(A) -- normal --> B -- estreito --> C
                                 |
                          elevador
                                 |
    Nível 1 (intermediário):
                                 D -- inundado --> E -- normal --> F
                                 |
                          elevador
                                 |
    Nível 2 (profundo):
                                 G -- instavel --> H -- normal --> I

    Colapsos programados:
    - Túnel (1,'D')-(1,'E') colapsa no step 12
    - Túnel (2,'G')-(2,'H') colapsa no step 18
    """
    mine_graph = {
        # Nível 0 (superfície)
        (0, 'A'): [((0, 'B'), 'normal')],
        (0, 'B'): [((0, 'A'), 'normal'), ((0, 'C'), 'estreito'), ((1, 'D'), 'elevador')],
        (0, 'C'): [((0, 'B'), 'estreito')],
        # Nível 1 (intermediário)
        (1, 'D'): [((0, 'B'), 'elevador'), ((1, 'E'), 'inundado'), ((2, 'G'), 'elevador')],
        (1, 'E'): [((1, 'D'), 'inundado'), ((1, 'F'), 'normal')],
        (1, 'F'): [((1, 'E'), 'normal')],
        # Nível 2 (profundo)
        (2, 'G'): [((1, 'D'), 'elevador'), ((2, 'H'), 'instavel')],
        (2, 'H'): [((2, 'G'), 'instavel'), ((2, 'I'), 'normal')],
        (2, 'I'): [((2, 'H'), 'normal')],
    }

    collapse_schedule = [
        ((1, 'D'), (1, 'E'), 12),  # Túnel inundado colapsa no step 12
        ((2, 'G'), (2, 'H'), 18),  # Túnel instável colapsa no step 18
    ]

    return mine_graph, collapse_schedule


def create_complex_mine():
    """
    Mina com rotas alternativas.
    Nível 0: A--B--C  |  Nível 1: D--E, D--F--J  |  Nível 2: G--H--I--K
    D->F é atalho instável (custo 4); A->B->C->F é mais barato (custo 4 total).
    """
    mine_graph = {
        # Nível 0
        (0, 'A'): [((0, 'B'), 'normal'), ((1, 'D'), 'elevador')],
        (0, 'B'): [((0, 'A'), 'normal'), ((0, 'C'), 'normal')],
        (0, 'C'): [((0, 'B'), 'normal'), ((1, 'F'), 'elevador')],
        # Nível 1
            (1, 'D'): [((0, 'A'), 'elevador'), ((1, 'E'), 'estreito'),
                   ((1, 'F'), 'instavel'), ((2, 'G'), 'elevador')],
        (1, 'E'): [((1, 'D'), 'estreito')],
        (1, 'F'): [((1, 'D'), 'instavel'), ((0, 'C'), 'elevador'),
                   ((1, 'J'), 'normal')],
        (1, 'J'): [((1, 'F'), 'normal'), ((2, 'K'), 'elevador')],
        # Nível 2
        (2, 'G'): [((1, 'D'), 'elevador'), ((2, 'H'), 'instavel')],
        (2, 'H'): [((2, 'G'), 'instavel'), ((2, 'I'), 'normal')],
        (2, 'I'): [((2, 'H'), 'normal'), ((2, 'K'), 'estreito')],
        (2, 'K'): [((2, 'I'), 'estreito'), ((1, 'J'), 'elevador')],
    }

    collapse_schedule = [
        ((1, 'D'), (1, 'F'), 14),
        ((2, 'G'), (2, 'H'), 18),
    ]

    return mine_graph, collapse_schedule
