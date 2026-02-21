"""
Problema de Resgate em Mina Subterrânea
========================================

Especificação Formal (conforme AIMA):

1. REPRESENTAÇÃO DOS ESTADOS:
   Estado = (posição_agente, frozenset(mineradores_resgatados), bateria, step)
   - posição_agente: tupla (nível, identificador) ex: (0, 'A')
   - mineradores_resgatados: conjunto imutável de localizações já resgatadas
   - bateria: int, energia restante do robô
   - step: int, passo atual (para calcular colapsos)

2. ESTADO INICIAL:
   (posição_entrada, frozenset(), bateria_inicial, 0)
   O agente começa na entrada da mina com bateria cheia e nenhum resgate.

3. CONJUNTO DE AÇÕES:
   - ('move', destino): Mover para nó adjacente via túnel não colapsado
   - ('rescue',): Resgatar minerador(es) na posição atual
   Ações são determinadas dinamicamente com base no estado atual.

4. MODELO DE TRANSIÇÃO - result(s, a):
   - move: atualiza posição, reduz bateria pelo custo do túnel, incrementa step
   - rescue: adiciona localização aos resgatados, reduz bateria em 1

5. TESTE DE OBJETIVO - goal_test(s):
   Todos os mineradores-alvo foram resgatados (target set ⊆ rescued set)

6. CUSTO DE CAMINHO - path_cost:
   Soma dos custos dos túneis percorridos + custos de resgate.
   Custos variam por tipo de túnel (normal=1, estreito=2, inundado=3, instável=4, elevador=2).

7. HEURÍSTICA h(n):
   h(n) = custo estimado mínimo para resgatar todos os mineradores restantes.
   Usa a distância Manhattan 3D adaptada ao grafo + penalidade por prioridade.
   - Admissível: h(n) ≤ custo real porque usa o mínimo custo possível por aresta (1).
   - Consistente: h(n) ≤ c(n,a,n') + h(n') pois a redução é limitada pelo custo da aresta.
"""

import sys
import os
from collections import deque

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'aima-python'))

from search import Problem, Node, astar_search, breadth_first_graph_search, \
    uniform_cost_search, depth_first_graph_search, iterative_deepening_search, \
    greedy_best_first_graph_search, best_first_graph_search


class MineRescueProblem(Problem):
    """
    Problema de busca para resgate em mina subterrânea.

    Subclasse de Problem (aima-python/search.py).

    O estado é representado como uma tupla:
        (posição, frozenset(resgatados), bateria, step)

    O goal é o conjunto de mineradores que devem ser resgatados.
    """

    def __init__(self, initial_location, miner_targets, environment,
                 battery, current_step=0):
        """
        Args:
            initial_location: Posição inicial do agente (nível, id)
            miner_targets: Lista de posições dos mineradores-alvo
            environment: Referência ao MineEnvironment para consultar o grafo
            battery: Bateria disponível
            current_step: Passo atual do ambiente
        """
        self.env = environment
        self.miner_targets = frozenset(miner_targets)

        # Estado: (posição, resgatados, bateria, step)
        initial_state = (initial_location, frozenset(), battery, current_step)

        super().__init__(initial=initial_state, goal=self.miner_targets)

        # Pré-calcular distâncias mínimas (BFS) para cada nó
        self._distance_cache = {}
        self._precompute_distances()

    def _precompute_distances(self):
        """Pré-calcula distâncias mínimas (em custo) entre todos os pares de nós
        usando BFS ponderado (Dijkstra simplificado)."""
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
        """
        Retorna as ações possíveis no estado atual.

        Mapeamento para o código:
        - Consulta os túneis não colapsados no passo atual
        - Verifica se há mineradores para resgatar na posição
        - Filtra ações inviáveis (bateria insuficiente)
        """
        position, rescued, battery, step = state

        possible_actions = []

        # Verificar quais túneis estão disponíveis no step atual
        for neighbor in self.env.adjacency.get(position, []):
            tunnel = self.env.tunnels.get((position, neighbor))
            if tunnel is None:
                continue

            # Verificar se colapsou neste step
            collapse_step = self.env.collapse_schedule.get((position, neighbor))
            if collapse_step is not None and step >= collapse_step:
                continue  # Túnel colapsado

            if not tunnel.collapsed and tunnel.cost <= battery:
                possible_actions.append(('move', neighbor))

        # Verificar se há mineradores para resgatar na posição atual
        unresc_miners = self.miner_targets - rescued
        if position in unresc_miners and battery >= 1:
            possible_actions.append(('rescue',))

        return possible_actions

    def result(self, state, action):
        """
        Modelo de transição: result(s, a) → s'

        Mapeamento para o código:
        - ('move', destino): atualiza posição e reduz bateria
        - ('rescue',): adiciona localização aos resgatados
        """
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
        """
        Teste de objetivo:
        Todos os mineradores-alvo foram resgatados.

        Mapeamento: verifica se o conjunto de resgatados contém todos os alvos.
        """
        _, rescued, _, _ = state
        return self.miner_targets.issubset(rescued)

    def path_cost(self, c, state1, action, state2):
        """
        Custo de caminho:
        Soma o custo real da ação (custo do túnel ou custo de resgate).

        Mapeamento: consulta o custo do túnel no grafo do ambiente.
        """
        if action[0] == 'move':
            position = state1[0]
            destination = action[1]
            tunnel = self.env.tunnels.get((position, destination))
            return c + (tunnel.cost if tunnel else 1)
        elif action[0] == 'rescue':
            return c + 1
        return c + 1

    def h(self, node):
        """
        Heurística para busca informada (A*, Greedy).

        h(n) = soma das distâncias mínimas do agente até cada minerador
               não resgatado, considerando custos reais dos túneis.

        Na verdade usamos: distância ao minerador mais distante não resgatado
        (admissível, pois é <= custo real de visitar todos).

        Admissibilidade:
        - Usa distâncias mínimas pré-calculadas (Dijkstra)
        - Nunca superestima porque o agente precisa ao menos percorrer
          a distância até o minerador mais distante

        Consistência:
        - h(n) - h(n') ≤ c(n, a, n') é satisfeita porque a distância
          diminui em no máximo o custo da aresta percorrida.

        Mapeamento: implementada como método h(node) da subclasse Problem,
        chamada automaticamente por astar_search.
        """
        state = node.state
        position, rescued, battery, step = state

        # Mineradores ainda não resgatados
        remaining = self.miner_targets - rescued

        if not remaining:
            return 0

        # Distância mínima ao minerador mais distante (admissível)
        max_dist = 0
        for miner_loc in remaining:
            dist = self._min_distance(position, miner_loc)
            if dist > max_dist:
                max_dist = dist

        return max_dist


class MineRescueMultiProblem(Problem):
    """
    Variante do problema que formula o resgate de múltiplos mineradores
    usando uma heurística MST (Minimum Spanning Tree) mais sofisticada.

    A heurística MST é mais informada: estima o custo mínimo de visitar
    todos os mineradores restantes como o custo de uma árvore geradora
    mínima sobre os nós {agente} ∪ {mineradores não resgatados}.

    Admissível: o custo da MST é um limite inferior do tour ótimo.
    """

    def __init__(self, initial_location, miner_targets, environment,
                 battery, current_step=0):
        self.env = environment
        self.miner_targets = frozenset(miner_targets)

        initial_state = (initial_location, frozenset(), battery, current_step)
        super().__init__(initial=initial_state, goal=self.miner_targets)

        self._distance_cache = {}
        self._precompute_distances()

    def _precompute_distances(self):
        """Dijkstra para todos os nós."""
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
        """
        Heurística MST (Minimum Spanning Tree).

        Calcula o custo da árvore geradora mínima sobre o conjunto
        {posição_atual} ∪ {mineradores não resgatados}.

        Admissível: MST ≤ custo ótimo do tour (TSP).
        Consistente: redução do MST ao mover ≤ custo da aresta.
        """
        state = node.state
        position, rescued, battery, step = state
        remaining = self.miner_targets - rescued

        if not remaining:
            return 0

        # Nós relevantes: posição atual + mineradores restantes
        nodes = [position] + list(remaining)

        if len(nodes) <= 1:
            return 0

        # Prim's MST
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
    """
    Problema simplificado: resgatar UM minerador específico.
    Usado quando o agente decide ir até um minerador de cada vez.

    Estado: (posição, bateria, step)
    Goal: posição do minerador-alvo
    """

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
        """
        Heurística: distância mínima pré-calculada até o alvo.
        Admissível e consistente (distâncias reais do grafo).
        """
        state = node.state
        position = state[0]
        return self._min_distance(position, self.target)
