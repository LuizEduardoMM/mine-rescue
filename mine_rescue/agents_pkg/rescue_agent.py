"""
Programa do Agente de Resgate
==============================

Implementa o Programa de Agente conforme a arquitetura AIMA:
  Ambiente → percepção → Programa de Agente → ação → Ambiente

O programa de agente NÃO é o algoritmo de busca.
Ele:
1. Recebe percepções do ambiente
2. Atualiza seu estado interno
3. Decide quando formular um problema
4. Executa busca para gerar um plano (sequência de ações)
5. Retorna UMA ação por passo ao ambiente

Baseado na classe SimpleProblemSolvingAgentProgram do aima-python (Figure 3.1).

Algoritmos de busca utilizados:
- A* Search (principal): Usa heurística admissível para encontrar caminho ótimo
- Uniform Cost Search: Usado como fallback quando A* falha
- Breadth-First Graph Search: Para verificar acessibilidade
- Greedy Best-First Search: Para decisões rápidas em emergências

Algoritmos NÃO utilizados e justificativa:
- DFS (Depth-First Search): Não garante solução ótima em grafos com custos
  variáveis; pode explorar caminhos muito longos desnecessariamente.
- Depth-Limited Search: Difícil definir limite adequado em mina com
  profundidade variável.
- Iterative Deepening: Muito lento para grafos com custos variáveis;
  re-expande muitos nós.
- Hill Climbing: Não é completo; pode ficar preso em máximos locais.
- Simulated Annealing: Para otimização, não para planejamento de caminho.
- Genetic Algorithm: Para otimização, não adequado para planejamento sequencial.
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'aima-python'))

from search import (SimpleProblemSolvingAgentProgram, astar_search,
                     uniform_cost_search, breadth_first_graph_search,
                     greedy_best_first_graph_search, best_first_graph_search)
from agents import Agent

# Importar nossas classes
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from problems.mine_problem import (MineRescueProblem, SingleMinerRescueProblem,
                                    MineRescueMultiProblem)
from env.mine_environment import RescueRobot, Miner


class MineRescueAgentProgram(SimpleProblemSolvingAgentProgram):
    """
    Programa do agente de resgate em mina subterrânea.

    Herda de SimpleProblemSolvingAgentProgram (aima-python, Figure 3.1).

    O agente:
    1. Recebe percepção → atualiza estado interno
    2. Se não tem plano → formula objetivo → formula problema → busca solução
    3. Executa uma ação do plano por passo
    4. Replanos quando necessário (colapsos, mudanças no ambiente)

    Estratégia de resgate:
    - Prioriza mineradores por urgência (prioridade + oxigênio restante)
    - Usa A* para encontrar caminho ótimo até cada minerador
    - Replanos se o caminho atual é invalidado por colapsos
    """

    def __init__(self, environment, search_algorithm='astar'):
        """
        Args:
            environment: Referência ao MineEnvironment
            search_algorithm: Algoritmo de busca a usar
                'astar' | 'ucs' | 'greedy' | 'bfs'
        """
        super().__init__()
        self.env = environment
        self.search_algorithm = search_algorithm
        self.current_target = None
        self.plan_invalid = False
        self.last_location = None
        self.stats = {
            'searches_performed': 0,
            'replans': 0,
            'actions_executed': 0,
            'algorithm_used': search_algorithm,
        }

    def __call__(self, percept):
        """
        Função principal do programa de agente.
        Recebe percepção e retorna uma ação.

        Fluxo (conforme Figure 3.1 do AIMA):
        1. state ← UPDATE-STATE(state, percept)
        2. if seq is empty then
        3.   goal ← FORMULATE-GOAL(state)
        4.   problem ← FORMULATE-PROBLEM(state, goal)
        5.   seq ← SEARCH(problem)
        6. if seq is empty then return None
        7. action ← FIRST(seq); seq ← REST(seq)
        8. return action
        """
        # 1. Atualizar estado interno com a percepção
        self.state = self.update_state(self.state, percept)

        # Verificar se precisa replanar
        if self._needs_replan(percept):
            self.seq = []
            self.plan_invalid = True
            self.stats['replans'] += 1

        # 2. Se não tem plano, formular e buscar
        if not self.seq:
            goal = self.formulate_goal(self.state)
            if goal is None:
                return None  # Nada mais para fazer

            problem = self.formulate_problem(self.state, goal)
            self.seq = self.search(problem)

            if not self.seq:
                # Busca falhou — tentar com algoritmo alternativo
                self.seq = self._fallback_search(problem)
                if not self.seq:
                    return None  # Sem solução possível

        # 3. Retornar próxima ação do plano
        if self.seq:
            action = self.seq.pop(0)
            self.stats['actions_executed'] += 1
            return action

        return None

    def update_state(self, state, percept):
        """
        Atualiza o estado interno do agente com base na percepção.

        Mapeamento: A percepção contém informações do ambiente (posição,
        túneis, mineradores, bateria). O agente mantém um modelo interno.
        """
        return {
            'location': percept['location'],
            'adjacent_tunnels': percept['adjacent_tunnels'],
            'miners_here': percept['miners_here'],
            'battery': percept['battery'],
            'step': percept['step'],
            'rescued_miners': percept['rescued_miners'],
            'all_miners': percept['all_miners'],
        }

    def formulate_goal(self, state):
        """
        Formula o objetivo com base no estado atual.

        Estratégia de priorização:
        1. Se há minerador na posição atual → resgatar imediatamente
        2. Senão → escolher o minerador mais urgente:
           - Ordenar por: prioridade (menor = mais urgente),
             depois por oxigênio (menor = mais urgente)

        Retorna: localização do minerador-alvo ou None
        """
        if state is None:
            return None

        # Se tem minerador na posição atual, resgatar
        if state['miners_here']:
            return state['location']

        # Selecionar minerador mais urgente
        active_miners = state.get('all_miners', [])
        if not active_miners:
            return None

        # Filtrar já resgatados
        rescued_locs = {m.location for m in state.get('rescued_miners', [])
                        if hasattr(m, 'location')}
        remaining = [m for m in active_miners if m['location'] not in rescued_locs]

        if not remaining:
            return None

        # Ordenar por urgência: prioridade ASC, oxigênio ASC
        remaining.sort(key=lambda m: (m['priority'], m['oxygen']))

        self.current_target = remaining[0]['location']
        return self.current_target

    def formulate_problem(self, state, goal):
        """
        Formula o problema de busca baseado no estado e objetivo.

        Se o objetivo é a posição atual → não precisa de busca de caminho
        Se o objetivo é outra posição → cria SingleMinerRescueProblem

        Mapeamento: Cria uma subclasse de Problem (aima-python) com as
        informações atuais do ambiente.
        """
        if state['location'] == goal:
            # Minerador na posição atual — ação direta
            return None

        return SingleMinerRescueProblem(
            initial_location=state['location'],
            target_location=goal,
            environment=self.env,
            battery=state['battery'],
            current_step=state['step']
        )

    def search(self, problem):
        """
        Executa o algoritmo de busca para encontrar sequência de ações.

        Usa o algoritmo configurado (A* por padrão).
        Retorna lista de ações ou lista vazia se falhar.

        Mapeamento: Chama os algoritmos do aima-python/search.py.
        Os algoritmos retornam um Node cuja .solution() dá as ações.
        """
        if problem is None:
            # Minerador na posição atual — ação direta de resgate
            return [('rescue',)]

        self.stats['searches_performed'] += 1

        try:
            if self.search_algorithm == 'astar':
                solution_node = astar_search(problem)
            elif self.search_algorithm == 'ucs':
                solution_node = uniform_cost_search(problem)
            elif self.search_algorithm == 'greedy':
                solution_node = greedy_best_first_graph_search(
                    problem, lambda n: problem.h(n))
            elif self.search_algorithm == 'bfs':
                solution_node = breadth_first_graph_search(problem)
            else:
                solution_node = astar_search(problem)

            if solution_node is None:
                return []

            # Extrair ações da solução + adicionar resgate no final
            actions = solution_node.solution()
            actions.append(('rescue',))
            return actions

        except Exception as e:
            print(f"  [Agente] Erro na busca: {e}")
            return []

    def _fallback_search(self, problem):
        """Tenta algoritmo alternativo se o principal falhar."""
        if problem is None:
            return [('rescue',)]

        try:
            # Tentar UCS como fallback
            solution_node = uniform_cost_search(problem)
            if solution_node:
                actions = solution_node.solution()
                actions.append(('rescue',))
                return actions
        except Exception:
            pass

        return []

    def _needs_replan(self, percept):
        """
        Verifica se o plano atual foi invalidado.

        Situações que exigem replanejamento:
        1. Ação de movimento falhou (agente não se moveu)
        2. Minerador-alvo já foi resgatado ou faleceu
        3. Caminho planejado contém túnel que colapsou
        """
        if not self.seq:
            return False

        location = percept['location']

        # Verificar se o movimento anterior falhou
        if self.last_location is not None and self.last_location == location:
            # Se tinha ação de mover mas não moveu → replanar
            if self.plan_invalid:
                self.plan_invalid = False
                return True

        self.last_location = location

        # Verificar se o alvo ainda existe
        if self.current_target:
            active_locs = [m['location'] for m in percept.get('all_miners', [])]
            if self.current_target not in active_locs:
                return True

        # Verificar se próxima ação é viável
        if self.seq:
            next_action = self.seq[0]
            if isinstance(next_action, tuple) and next_action[0] == 'move':
                dest = next_action[1]
                adjacent_dests = [t['destination']
                                  for t in percept['adjacent_tunnels']]
                if dest not in adjacent_dests:
                    return True  # Túnel colapsou

        return False

    def get_stats(self):
        """Retorna estatísticas de desempenho do agente."""
        return self.stats


class MineRescueMultiAgentProgram(SimpleProblemSolvingAgentProgram):
    """
    Variante que resolve o resgate de TODOS os mineradores de uma vez,
    usando MineRescueProblem (busca com múltiplos objetivos).

    Mais eficiente para cenários onde a ordem de resgate importa,
    mas mais custoso computacionalmente.
    """

    def __init__(self, environment, search_algorithm='astar'):
        super().__init__()
        self.env = environment
        self.search_algorithm = search_algorithm
        self.stats = {
            'searches_performed': 0,
            'replans': 0,
            'actions_executed': 0,
            'algorithm_used': search_algorithm + '_multi',
        }

    def __call__(self, percept):
        self.state = self.update_state(self.state, percept)

        if not self.seq:
            goal = self.formulate_goal(self.state)
            if goal is None:
                return None
            problem = self.formulate_problem(self.state, goal)
            if problem is None:
                return None
            self.seq = self.search(problem)
            if not self.seq:
                return None

        if self.seq:
            action = self.seq.pop(0)
            self.stats['actions_executed'] += 1
            return action
        return None

    def update_state(self, state, percept):
        return {
            'location': percept['location'],
            'adjacent_tunnels': percept['adjacent_tunnels'],
            'miners_here': percept['miners_here'],
            'battery': percept['battery'],
            'step': percept['step'],
            'rescued_miners': percept['rescued_miners'],
            'all_miners': percept['all_miners'],
        }

    def formulate_goal(self, state):
        active_miners = state.get('all_miners', [])
        if not active_miners:
            return None
        rescued_locs = {m.location for m in state.get('rescued_miners', [])
                        if hasattr(m, 'location')}
        targets = [m['location'] for m in active_miners
                   if m['location'] not in rescued_locs]
        return targets if targets else None

    def formulate_problem(self, state, goal):
        if not goal:
            return None
        return MineRescueProblem(
            initial_location=state['location'],
            miner_targets=goal,
            environment=self.env,
            battery=state['battery'],
            current_step=state['step']
        )

    def search(self, problem):
        self.stats['searches_performed'] += 1
        try:
            if self.search_algorithm == 'astar':
                solution_node = astar_search(problem)
            elif self.search_algorithm == 'ucs':
                solution_node = uniform_cost_search(problem)
            else:
                solution_node = astar_search(problem)

            if solution_node is None:
                return []
            return solution_node.solution()
        except Exception as e:
            print(f"  [AgenteMulti] Erro na busca: {e}")
            return []

    def get_stats(self):
        return self.stats


def create_rescue_agent(environment, search_algorithm='astar', battery=50):
    """
    Factory: cria um RescueRobot com o programa de agente configurado.

    Args:
        environment: MineEnvironment
        search_algorithm: 'astar' | 'ucs' | 'greedy' | 'bfs'
        battery: bateria inicial

    Returns:
        RescueRobot com programa de agente configurado
    """
    program = MineRescueAgentProgram(environment, search_algorithm)
    robot = RescueRobot(program=program, battery=battery)
    return robot


def create_multi_rescue_agent(environment, search_algorithm='astar', battery=50):
    """
    Factory: cria um RescueRobot com programa de resgate multi-objetivo.
    """
    program = MineRescueMultiAgentProgram(environment, search_algorithm)
    robot = RescueRobot(program=program, battery=battery)
    return robot
