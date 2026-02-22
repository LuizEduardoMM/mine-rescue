"""
Problemas de busca para resgate em mina.
Subclasses de Problem (aima-python/search.py).

Estado = (posição, resgatados, bateria, step)
Ações = ('move', destino) | ('rescue',)
"""

import sys
import os
from collections import deque

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'aima-python'))

from search import Problem, Node, astar_search, breadth_first_graph_search, \
    uniform_cost_search, depth_first_graph_search, iterative_deepening_search, \
    greedy_best_first_graph_search, best_first_graph_search


class MineRescueProblem(Problem):
    """Problema de busca multi-objetivo: resgatar todos os mineradores."""

    def __init__(self, initial_location, miner_targets, environment,
                 battery, current_step=0):
        self.env = environment
        self.miner_targets = frozenset(miner_targets)

        initial_state = (initial_location, frozenset(), battery, current_step)
        super().__init__(initial=initial_state, goal=self.miner_targets)
        self._distance_cache = {}
        self._precompute_distances()

    def _precompute_distances(self):
        """Dijkstra para pré-calcular distâncias mínimas entre todos os nós."""
        all_nodes = self.env.get_all_nodes()
        for source in all_nodes:
            distances = {source: 0}
            queue = [(0, source)]
            import heapq
            heapq.heapify(queue)
            while queue:
                cost, node = heapq.heappop(queue)
                if cost > distances.get(node, float('inf')):
                    continue
                for neighbor, tunnel_cost, _ in self.env.get_available_neighbors(node):
                    new_cost = cost + tunnel_cost
                    if new_cost < distances.get(neighbor, float('inf')):
                        distances[neighbor] = new_cost
                        heapq.heappush(queue, (new_cost, neighbor))
            self._distance_cache[source] = distances

    def _min_distance(self, from_node, to_node):
        """Retorna a distância mínima pré-calculada entre dois nós."""
        if from_node in self._distance_cache:
            return self._distance_cache[from_node].get(to_node, float('inf'))
        return float('inf')

    def actions(self, state):
        """Ações possíveis: mover por túneis disponíveis ou resgatar."""
        position, rescued, battery, step = state

        possible_actions = []

        for neighbor in self.env.adjacency.get(position, []):
            tunnel = self.env.tunnels.get((position, neighbor))
            if tunnel is None:
                continue

            collapse_step = self.env.collapse_schedule.get((position, neighbor))
            if collapse_step is not None and step >= collapse_step:
                continue

            if not tunnel.collapsed and tunnel.cost <= battery:
                possible_actions.append(('move', neighbor))

        unresc_miners = self.miner_targets - rescued
        if position in unresc_miners and battery >= 1:
            possible_actions.append(('rescue',))

        return possible_actions

    def result(self, state, action):
        """Transição: move atualiza posição/bateria, rescue adiciona aos resgatados."""
        position, rescued, battery, step = state

        if action[0] == 'move':
            destination = action[1]
            tunnel = self.env.tunnels.get((position, destination))
            cost = tunnel.cost if tunnel else 1
            new_step = step + 1
            return (destination, rescued, battery - cost, new_step)

        elif action[0] == 'rescue':
            new_rescued = rescued | {position}
            new_step = step + 1
            return (position, new_rescued, battery - 1, new_step)

        return state

    def goal_test(self, state):
        """Verifica se todos os alvos foram resgatados."""
        _, rescued, _, _ = state
        return self.miner_targets.issubset(rescued)

    def path_cost(self, c, state1, action, state2):
        """Soma custo do túnel (ou 1 para resgate) ao custo acumulado."""
        if action[0] == 'move':
            position = state1[0]
            destination = action[1]
            tunnel = self.env.tunnels.get((position, destination))
            return c + (tunnel.cost if tunnel else 1)
        elif action[0] == 'rescue':
            return c + 1
        return c + 1

    def h(self, node):
        """Heurística: distância Dijkstra ao minerador mais distante. Admissível e consistente."""
        state = node.state
        position, rescued, battery, step = state

        remaining = self.miner_targets - rescued

        if not remaining:
            return 0

        max_dist = 0
        for miner_loc in remaining:
            dist = self._min_distance(position, miner_loc)
            if dist > max_dist:
                max_dist = dist

        return max_dist


class MineRescueMultiProblem(Problem):
    """Variante com heurística MST para múltiplos mineradores."""

    def __init__(self, initial_location, miner_targets, environment,
                 battery, current_step=0):
        self.env = environment
        self.miner_targets = frozenset(miner_targets)

        initial_state = (initial_location, frozenset(), battery, current_step)
        super().__init__(initial=initial_state, goal=self.miner_targets)

        self._distance_cache = {}
        self._precompute_distances()

    def _precompute_distances(self):
        all_nodes = self.env.get_all_nodes()
        for source in all_nodes:
            distances = {source: 0}
            import heapq
            queue = [(0, source)]
            heapq.heapify(queue)
            while queue:
                cost, node = heapq.heappop(queue)
                if cost > distances.get(node, float('inf')):
                    continue
                for neighbor, tunnel_cost, _ in self.env.get_available_neighbors(node):
                    new_cost = cost + tunnel_cost
                    if new_cost < distances.get(neighbor, float('inf')):
                        distances[neighbor] = new_cost
                        heapq.heappush(queue, (new_cost, neighbor))
            self._distance_cache[source] = distances

    def _min_distance(self, from_node, to_node):
        if from_node in self._distance_cache:
            return self._distance_cache[from_node].get(to_node, float('inf'))
        return float('inf')

    def actions(self, state):
        position, rescued, battery, step = state
        possible_actions = []

        for neighbor in self.env.adjacency.get(position, []):
            tunnel = self.env.tunnels.get((position, neighbor))
            if tunnel is None:
                continue
            collapse_step = self.env.collapse_schedule.get((position, neighbor))
            if collapse_step is not None and step >= collapse_step:
                continue
            if not tunnel.collapsed and tunnel.cost <= battery:
                possible_actions.append(('move', neighbor))

        unresc = self.miner_targets - rescued
        if position in unresc and battery >= 1:
            possible_actions.append(('rescue',))

        return possible_actions

    def result(self, state, action):
        position, rescued, battery, step = state

        if action[0] == 'move':
            destination = action[1]
            tunnel = self.env.tunnels.get((position, destination))
            cost = tunnel.cost if tunnel else 1
            return (destination, rescued, battery - cost, step + 1)

        elif action[0] == 'rescue':
            return (position, rescued | {position}, battery - 1, step + 1)

        return state

    def goal_test(self, state):
        _, rescued, _, _ = state
        return self.miner_targets.issubset(rescued)

    def path_cost(self, c, state1, action, state2):
        if action[0] == 'move':
            position = state1[0]
            destination = action[1]
            tunnel = self.env.tunnels.get((position, destination))
            return c + (tunnel.cost if tunnel else 1)
        elif action[0] == 'rescue':
            return c + 1
        return c + 1

    def h(self, node):
        """Heurística MST: custo da árvore geradora mínima sobre os nós restantes."""
        state = node.state
        position, rescued, battery, step = state
        remaining = self.miner_targets - rescued

        if not remaining:
            return 0

        nodes = [position] + list(remaining)

        if len(nodes) <= 1:
            return 0

        import heapq
        in_mst = {nodes[0]}
        edges = []
        for other in nodes[1:]:
            d = self._min_distance(nodes[0], other)
            heapq.heappush(edges, (d, other))

        mst_cost = 0
        while edges and len(in_mst) < len(nodes):
            cost, node_n = heapq.heappop(edges)
            if node_n in in_mst:
                continue
            in_mst.add(node_n)
            mst_cost += cost
            for other in nodes:
                if other not in in_mst:
                    d = self._min_distance(node_n, other)
                    heapq.heappush(edges, (d, other))

        return mst_cost


class SingleMinerRescueProblem(Problem):
    """Busca caminho até um único minerador. Estado: (posição, bateria, step)."""

    def __init__(self, initial_location, target_location, environment,
                 battery, current_step=0):
        self.env = environment
        self.target = target_location

        initial_state = (initial_location, battery, current_step)
        super().__init__(initial=initial_state, goal=target_location)

        self._distance_cache = {}
        self._precompute_distances()

    def _precompute_distances(self):
        all_nodes = self.env.get_all_nodes()
        for source in all_nodes:
            distances = {source: 0}
            import heapq
            queue = [(0, source)]
            heapq.heapify(queue)
            while queue:
                cost, node_n = heapq.heappop(queue)
                if cost > distances.get(node_n, float('inf')):
                    continue
                for neighbor, tunnel_cost, _ in self.env.get_available_neighbors(node_n):
                    new_cost = cost + tunnel_cost
                    if new_cost < distances.get(neighbor, float('inf')):
                        distances[neighbor] = new_cost
                        heapq.heappush(queue, (new_cost, neighbor))
            self._distance_cache[source] = distances

    def _min_distance(self, from_node, to_node):
        if from_node in self._distance_cache:
            return self._distance_cache[from_node].get(to_node, float('inf'))
        return float('inf')

    def actions(self, state):
        position, battery, step = state
        possible_actions = []

        for neighbor in self.env.adjacency.get(position, []):
            tunnel = self.env.tunnels.get((position, neighbor))
            if tunnel is None:
                continue
            collapse_step_val = self.env.collapse_schedule.get((position, neighbor))
            if collapse_step_val is not None and step >= collapse_step_val:
                continue
            if not tunnel.collapsed and tunnel.cost <= battery:
                possible_actions.append(('move', neighbor))

        return possible_actions

    def result(self, state, action):
        position, battery, step = state
        if action[0] == 'move':
            destination = action[1]
            tunnel = self.env.tunnels.get((position, destination))
            cost = tunnel.cost if tunnel else 1
            return (destination, battery - cost, step + 1)
        return state

    def goal_test(self, state):
        position, _, _ = state
        return position == self.target

    def path_cost(self, c, state1, action, state2):
        if action[0] == 'move':
            position = state1[0]
            destination = action[1]
            tunnel = self.env.tunnels.get((position, destination))
            return c + (tunnel.cost if tunnel else 1)
        return c + 1

    def h(self, node):
        """Distância Dijkstra até o alvo (admissível e consistente)."""
        state = node.state
        position = state[0]
        return self._min_distance(position, self.target)
