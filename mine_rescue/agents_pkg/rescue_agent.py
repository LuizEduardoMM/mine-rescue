"""
Programa do agente de resgate.
Herda de SimpleProblemSolvingAgentProgram (Figure 3.1 do AIMA).
Recebe percepções, formula problema, executa busca e retorna ações.
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'aima-python'))

from search import (SimpleProblemSolvingAgentProgram, astar_search,
                     uniform_cost_search, breadth_first_graph_search,
                     greedy_best_first_graph_search, best_first_graph_search)
from agents import Agent


class _CountingWrapper:
    """Wrapper que conta chamadas a actions() = nós expandidos."""
    def __init__(self, problem):
        self._problem = problem
        self.nodes_expanded = 0

    def actions(self, state):
        self.nodes_expanded += 1
        return self._problem.actions(state)

    def result(self, state, action):
        return self._problem.result(state, action)

    def goal_test(self, state):
        return self._problem.goal_test(state)

    def path_cost(self, c, s1, a, s2):
        return self._problem.path_cost(c, s1, a, s2)

    def h(self, node):
        return self._problem.h(node)

    def value(self, state):
        return self._problem.value(state) if hasattr(self._problem, 'value') else 0

    @property
    def initial(self):
        return self._problem.initial

    @property
    def goal(self):
        return self._problem.goal

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from problems.mine_problem import (MineRescueProblem, SingleMinerRescueProblem,
                                    MineRescueMultiProblem)
from env.mine_environment import RescueRobot, Miner


class MineRescueAgentProgram(SimpleProblemSolvingAgentProgram):
    """Programa do agente. Prioriza mineradores por urgência e replana em colapsos."""

    def __init__(self, environment, search_algorithm='astar'):
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
            'nodes_expanded': 0,
        }

    def __call__(self, percept):
        """Recebe percepção, atualiza estado, formula/busca se preciso, retorna ação."""
        self.state = self.update_state(self.state, percept)
        if self._needs_replan(percept):
            self.seq = []
            self.plan_invalid = True
            self.stats['replans'] += 1

        if not self.seq:
            goal = self.formulate_goal(self.state)
            if goal is None:
                return None

            problem = self.formulate_problem(self.state, goal)
            self.seq = self.search(problem)

            if not self.seq:
                self.seq = self._fallback_search(problem)
                if not self.seq:
                    return None

        if self.seq:
            action = self.seq.pop(0)
            self.stats['actions_executed'] += 1
            return action

        return None

    def update_state(self, state, percept):
        """Atualiza estado interno do agente a partir da percepção."""
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
        """Escolhe o próximo alvo: minerador aqui ou o mais urgente."""
        if state is None:
            return None

        if state['miners_here']:
            return state['location']

        active_miners = state.get('all_miners', [])
        if not active_miners:
            return None

        rescued_locs = {m.location for m in state.get('rescued_miners', [])
                        if hasattr(m, 'location')}
        remaining = [m for m in active_miners if m['location'] not in rescued_locs]

        if not remaining:
            return None

        remaining.sort(key=lambda m: (m['priority'], m['oxygen']))

        self.current_target = remaining[0]['location']
        return self.current_target

    def formulate_problem(self, state, goal):
        """Cria um SingleMinerRescueProblem para o alvo, ou None se já está aqui."""
        if state['location'] == goal:
            return None

        return SingleMinerRescueProblem(
            initial_location=state['location'],
            target_location=goal,
            environment=self.env,
            battery=state['battery'],
            current_step=state['step']
        )

    def search(self, problem):
        """Executa o algoritmo de busca configurado. Retorna lista de ações."""
        if problem is None:
            return [('rescue',)]

        self.stats['searches_performed'] += 1
        counted = _CountingWrapper(problem)

        try:
            if self.search_algorithm == 'astar':
                solution_node = astar_search(counted)
            elif self.search_algorithm == 'ucs':
                solution_node = uniform_cost_search(counted)
            elif self.search_algorithm == 'greedy':
                solution_node = greedy_best_first_graph_search(
                    counted, lambda n: counted.h(n))
            elif self.search_algorithm == 'bfs':
                solution_node = breadth_first_graph_search(counted)
            else:
                solution_node = astar_search(counted)

            self.stats['nodes_expanded'] += counted.nodes_expanded

            if solution_node is None:
                return []

            actions = solution_node.solution()
            actions.append(('rescue',))
            return actions

        except Exception:
            self.stats['nodes_expanded'] += counted.nodes_expanded
            return []

    def _fallback_search(self, problem):
        if problem is None:
            return [('rescue',)]

        try:
            solution_node = uniform_cost_search(problem)
            if solution_node:
                actions = solution_node.solution()
                actions.append(('rescue',))
                return actions
        except Exception:
            pass

        return []

    def _needs_replan(self, percept):
        """Verifica se precisa replanar (túnel colapsou, alvo sumiu, etc)."""
        if not self.seq:
            return False

        location = percept['location']

        if self.last_location is not None and self.last_location == location:
            if self.plan_invalid:
                self.plan_invalid = False
                return True

        self.last_location = location

        if self.current_target:
            active_locs = [m['location'] for m in percept.get('all_miners', [])]
            if self.current_target not in active_locs:
                return True

        if self.seq:
            next_action = self.seq[0]
            if isinstance(next_action, tuple) and next_action[0] == 'move':
                dest = next_action[1]
                adjacent_dests = [t['destination']
                                  for t in percept['adjacent_tunnels']]
                if dest not in adjacent_dests:
                    return True

        return False

    def get_stats(self):
        return self.stats


class MineRescueMultiAgentProgram(SimpleProblemSolvingAgentProgram):
    """Variante multi-objetivo: planeja o resgate de todos os mineradores de uma vez."""

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
            pass
            return []

    def get_stats(self):
        return self.stats


def create_rescue_agent(environment, search_algorithm='astar', battery=50):
    """Cria um RescueRobot com o programa de agente configurado."""
    program = MineRescueAgentProgram(environment, search_algorithm)
    robot = RescueRobot(program=program, battery=battery)
    return robot


def create_multi_rescue_agent(environment, search_algorithm='astar', battery=50):
    """Cria um RescueRobot com programa multi-objetivo."""
    program = MineRescueMultiAgentProgram(environment, search_algorithm)
    robot = RescueRobot(program=program, battery=battery)
    return robot
