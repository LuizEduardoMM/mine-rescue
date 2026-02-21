"""
Testes automatizados para o projeto Mine Rescue Agent.

Testa:
1. Ambiente (MineEnvironment)
2. Problema de busca (MineRescueProblem, SingleMinerRescueProblem)
3. Agente (MineRescueAgentProgram)
4. Integração completa

Executar com: pytest tests/test_mine_rescue.py -v
"""

import sys
import os
import pytest

# Configurar paths — aima-python PRIMEIRO para que 'agents' resolva para aima-python/agents.py
# e nossos módulos usem nomes qualificados
_project_dir = os.path.join(os.path.dirname(__file__), '..')
_aima_dir = os.path.join(_project_dir, '..', 'aima-python')
sys.path.insert(0, os.path.abspath(_aima_dir))
sys.path.insert(0, os.path.abspath(_project_dir))

from env.mine_environment import (MineEnvironment, Miner, RescueRobot,
                                   Tunnel, Debris, TUNNEL_COSTS,
                                   create_default_mine, create_complex_mine)
from problems.mine_problem import (MineRescueProblem, SingleMinerRescueProblem,
                                    MineRescueMultiProblem)
from agents_pkg.rescue_agent import (MineRescueAgentProgram, create_rescue_agent,
                                      create_multi_rescue_agent)
from search import astar_search, uniform_cost_search, breadth_first_graph_search


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def simple_mine():
    """Mina simples para testes unitários: A -- B -- C (todos normal)."""
    mine_graph = {
        (0, 'A'): [((0, 'B'), 'normal')],
        (0, 'B'): [((0, 'A'), 'normal'), ((0, 'C'), 'normal')],
        (0, 'C'): [((0, 'B'), 'normal')],
    }
    return MineEnvironment(mine_graph)


@pytest.fixture
def default_mine():
    """Mina padrão com 3 níveis."""
    mine_graph, collapse_schedule = create_default_mine()
    env = MineEnvironment(mine_graph, collapse_schedule)
    env.max_steps = 60
    return env


@pytest.fixture
def complex_mine():
    """Mina complexa com rotas alternativas."""
    mine_graph, collapse_schedule = create_complex_mine()
    env = MineEnvironment(mine_graph, collapse_schedule)
    env.max_steps = 80
    return env


# =============================================================================
# Testes do Ambiente (Environment)
# =============================================================================

class TestMineEnvironment:
    """Testes para o ambiente da mina."""

    def test_environment_creation(self, simple_mine):
        """Ambiente deve ser criado com o grafo correto."""
        env = simple_mine
        assert len(env.adjacency) == 3
        assert (0, 'A') in env.adjacency
        assert (0, 'B') in env.adjacency
        assert (0, 'C') in env.adjacency

    def test_adjacency(self, simple_mine):
        """Verificar adjacências corretas."""
        env = simple_mine
        # A é adjacente a B
        assert (0, 'B') in env.adjacency[(0, 'A')]
        # B é adjacente a A e C
        assert (0, 'A') in env.adjacency[(0, 'B')]
        assert (0, 'C') in env.adjacency[(0, 'B')]
        # C é adjacente a B
        assert (0, 'B') in env.adjacency[(0, 'C')]

    def test_tunnel_creation(self, simple_mine):
        """Túneis devem ser criados bidirecionalmente."""
        env = simple_mine
        assert ((0, 'A'), (0, 'B')) in env.tunnels
        assert ((0, 'B'), (0, 'A')) in env.tunnels

    def test_tunnel_costs(self):
        """Custos de túneis devem corresponder ao tipo."""
        assert TUNNEL_COSTS['normal'] == 1
        assert TUNNEL_COSTS['estreito'] == 2
        assert TUNNEL_COSTS['inundado'] == 3
        assert TUNNEL_COSTS['instavel'] == 4
        assert TUNNEL_COSTS['elevador'] == 2

    def test_add_miner(self, simple_mine):
        """Deve ser possível adicionar minerador ao ambiente."""
        env = simple_mine
        miner = Miner(priority=1, oxygen=20)
        env.add_thing(miner, location=(0, 'C'))
        assert miner in env.things
        assert miner.location == (0, 'C')

    def test_add_robot(self, simple_mine):
        """Deve ser possível adicionar robô ao ambiente."""
        env = simple_mine
        robot = RescueRobot(battery=50)
        env.add_thing(robot, location=(0, 'A'))
        assert robot in env.agents
        assert robot.location == (0, 'A')

    def test_miner_oxygen_decreases(self):
        """Oxigênio do minerador deve diminuir a cada tick."""
        miner = Miner(priority=1, oxygen=5)
        assert miner.oxygen == 5
        miner.tick()
        assert miner.oxygen == 4
        miner.tick()
        assert miner.oxygen == 3

    def test_miner_dies_without_oxygen(self):
        """Minerador deve falecer quando oxigênio acaba."""
        miner = Miner(priority=1, oxygen=2)
        assert miner.alive is True
        miner.tick()  # oxygen = 1
        assert miner.alive is True
        miner.tick()  # oxygen = 0
        assert miner.alive is False

    def test_percept(self, simple_mine):
        """Percepção deve conter informações corretas."""
        env = simple_mine
        robot = RescueRobot(battery=30)
        env.add_thing(robot, location=(0, 'A'))

        percept = env.percept(robot)
        assert percept['location'] == (0, 'A')
        assert percept['battery'] == 30
        assert percept['step'] == 0
        assert len(percept['adjacent_tunnels']) == 1  # Só B
        assert percept['adjacent_tunnels'][0]['destination'] == (0, 'B')

    def test_execute_move(self, simple_mine):
        """Agente deve conseguir se mover para nó adjacente."""
        env = simple_mine
        robot = RescueRobot(battery=30)
        env.add_thing(robot, location=(0, 'A'))

        env.execute_action(robot, ('move', (0, 'B')))
        assert robot.location == (0, 'B')
        assert robot.battery == 29  # normal cost = 1

    def test_execute_move_costly(self):
        """Movimento por túnel estreito deve custar mais."""
        mine_graph = {
            (0, 'A'): [((0, 'B'), 'estreito')],
            (0, 'B'): [((0, 'A'), 'estreito')],
        }
        env = MineEnvironment(mine_graph)
        robot = RescueRobot(battery=30)
        env.add_thing(robot, location=(0, 'A'))

        env.execute_action(robot, ('move', (0, 'B')))
        assert robot.location == (0, 'B')
        assert robot.battery == 28  # estreito cost = 2

    def test_execute_rescue(self, simple_mine):
        """Agente deve conseguir resgatar minerador na mesma posição."""
        env = simple_mine
        miner = Miner(priority=1, oxygen=10)
        env.add_thing(miner, location=(0, 'A'))
        robot = RescueRobot(battery=30)
        env.add_thing(robot, location=(0, 'A'))

        env.execute_action(robot, ('rescue',))
        assert miner.rescued is True
        assert len(robot.rescued_miners) == 1

    def test_collapse_schedule(self, default_mine):
        """Túneis devem colapsar no step programado."""
        env = default_mine
        robot = RescueRobot(battery=100)
        env.add_thing(robot, location=(0, 'A'))

        # Antes do colapso, túnel D-E deve estar aberto
        tunnel = env.tunnels.get(((1, 'D'), (1, 'E')))
        assert tunnel is not None
        assert tunnel.collapsed is False

        # Simular passos até o colapso
        for _ in range(12):
            env.exogenous_change()

        assert tunnel.collapsed is True

    def test_is_done_all_rescued(self, simple_mine):
        """Simulação deve terminar quando todos mineradores forem resgatados."""
        env = simple_mine
        miner = Miner(priority=1, oxygen=10)
        env.add_thing(miner, location=(0, 'A'))
        robot = RescueRobot(battery=30)
        env.add_thing(robot, location=(0, 'A'))

        miner.rescued = True
        assert env.is_done() is True

    def test_is_done_agent_dead(self, simple_mine):
        """Simulação deve terminar quando agente fica sem bateria."""
        env = simple_mine
        miner = Miner(priority=1, oxygen=50)
        env.add_thing(miner, location=(0, 'C'))
        robot = RescueRobot(battery=0)
        env.add_thing(robot, location=(0, 'A'))

        robot.alive = False
        assert env.is_done() is True

    def test_get_available_neighbors(self, simple_mine):
        """Deve retornar vizinhos disponíveis corretamente."""
        env = simple_mine
        neighbors = env.get_available_neighbors((0, 'B'))
        assert len(neighbors) == 2
        destinations = [n[0] for n in neighbors]
        assert (0, 'A') in destinations
        assert (0, 'C') in destinations

    def test_get_tunnel_cost(self, simple_mine):
        """Deve retornar custo correto do túnel."""
        env = simple_mine
        cost = env.get_tunnel_cost((0, 'A'), (0, 'B'))
        assert cost == 1

    def test_render_no_error(self, default_mine):
        """Render não deve lançar exceção."""
        env = default_mine
        robot = RescueRobot(battery=50)
        env.add_thing(robot, location=(0, 'A'))
        env.add_thing(Miner(priority=1, oxygen=10), location=(1, 'F'))
        # Apenas verificar que não lança exceção
        env.render()

    def test_multilevel_mine(self, default_mine):
        """Mina multinível deve ter nós em diferentes níveis."""
        env = default_mine
        all_nodes = env.get_all_nodes()
        levels = set(n[0] for n in all_nodes)
        assert 0 in levels
        assert 1 in levels
        assert 2 in levels

    def test_elevator_between_levels(self, default_mine):
        """Elevadores devem conectar níveis diferentes."""
        env = default_mine
        # B (nível 0) → D (nível 1) via elevador
        tunnel = env.tunnels.get(((0, 'B'), (1, 'D')))
        assert tunnel is not None
        assert tunnel.tunnel_type == 'elevador'
        assert tunnel.cost == 2


# =============================================================================
# Testes do Problema de Busca (Problem)
# =============================================================================

class TestMineRescueProblem:
    """Testes para o problema de busca."""

    def test_initial_state(self, simple_mine):
        """Estado inicial deve conter posição, resgatados vazios, bateria e step."""
        env = simple_mine
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'A'),
            target_location=(0, 'C'),
            environment=env,
            battery=30
        )
        pos, bat, step = problem.initial
        assert pos == (0, 'A')
        assert bat == 30
        assert step == 0

    def test_actions(self, simple_mine):
        """Ações devem ser geradas corretamente para o estado."""
        env = simple_mine
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'A'),
            target_location=(0, 'C'),
            environment=env,
            battery=30
        )
        actions = problem.actions(problem.initial)
        assert len(actions) == 1  # Só pode mover para B
        assert actions[0] == ('move', (0, 'B'))

    def test_result(self, simple_mine):
        """Modelo de transição deve atualizar estado corretamente."""
        env = simple_mine
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'A'),
            target_location=(0, 'C'),
            environment=env,
            battery=30
        )
        state = problem.initial
        new_state = problem.result(state, ('move', (0, 'B')))
        pos, bat, step = new_state
        assert pos == (0, 'B')
        assert bat == 29  # custo 1
        assert step == 1

    def test_goal_test(self, simple_mine):
        """Teste de objetivo deve funcionar corretamente."""
        env = simple_mine
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'A'),
            target_location=(0, 'C'),
            environment=env,
            battery=30
        )
        # Estado inicial não é goal
        assert problem.goal_test(problem.initial) is False
        # Estado no objetivo
        goal_state = ((0, 'C'), 28, 2)
        assert problem.goal_test(goal_state) is True

    def test_path_cost(self, simple_mine):
        """Custo de caminho deve somar custos dos túneis."""
        env = simple_mine
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'A'),
            target_location=(0, 'C'),
            environment=env,
            battery=30
        )
        cost = problem.path_cost(0, ((0, 'A'), 30, 0),
                                  ('move', (0, 'B')),
                                  ((0, 'B'), 29, 1))
        assert cost == 1  # normal tunnel

    def test_heuristic_admissible(self, simple_mine):
        """Heurística deve ser admissível (nunca superestimar)."""
        env = simple_mine
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'A'),
            target_location=(0, 'C'),
            environment=env,
            battery=30
        )
        from search import Node
        node = Node(state=problem.initial)
        h_value = problem.h(node)
        # Caminho real A→B→C tem custo 2, h deve ser ≤ 2
        assert h_value <= 2

    def test_heuristic_at_goal_is_zero(self, simple_mine):
        """Heurística no objetivo deve ser 0."""
        env = simple_mine
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'C'),
            target_location=(0, 'C'),
            environment=env,
            battery=30
        )
        from search import Node
        node = Node(state=problem.initial)
        assert problem.h(node) == 0

    def test_astar_finds_solution(self, simple_mine):
        """A* deve encontrar solução para problema simples."""
        env = simple_mine
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'A'),
            target_location=(0, 'C'),
            environment=env,
            battery=30
        )
        solution = astar_search(problem)
        assert solution is not None
        actions = solution.solution()
        assert len(actions) == 2  # A→B, B→C
        assert actions[0] == ('move', (0, 'B'))
        assert actions[1] == ('move', (0, 'C'))

    def test_ucs_finds_solution(self, simple_mine):
        """UCS deve encontrar solução para problema simples."""
        env = simple_mine
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'A'),
            target_location=(0, 'C'),
            environment=env,
            battery=30
        )
        solution = uniform_cost_search(problem)
        assert solution is not None
        actions = solution.solution()
        assert len(actions) == 2

    def test_bfs_finds_solution(self, simple_mine):
        """BFS deve encontrar solução para problema simples."""
        env = simple_mine
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'A'),
            target_location=(0, 'C'),
            environment=env,
            battery=30
        )
        solution = breadth_first_graph_search(problem)
        assert solution is not None

    def test_multi_miner_problem(self, simple_mine):
        """Problema multi-minerador deve funcionar."""
        env = simple_mine
        env.add_thing(Miner(priority=1, oxygen=20), location=(0, 'B'))
        env.add_thing(Miner(priority=2, oxygen=20), location=(0, 'C'))

        problem = MineRescueProblem(
            initial_location=(0, 'A'),
            miner_targets=[(0, 'B'), (0, 'C')],
            environment=env,
            battery=30
        )
        solution = astar_search(problem)
        assert solution is not None
        assert problem.goal_test(solution.state) is True

    def test_no_solution_when_unreachable(self):
        """Deve retornar None quando minerador é inacessível."""
        mine_graph = {
            (0, 'A'): [((0, 'B'), 'normal')],
            (0, 'B'): [((0, 'A'), 'normal')],
            (0, 'C'): [],  # Isolado
        }
        env = MineEnvironment(mine_graph)
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'A'),
            target_location=(0, 'C'),
            environment=env,
            battery=30
        )
        solution = astar_search(problem)
        assert solution is None

    def test_optimal_path_with_costs(self):
        """A* deve preferir caminho mais barato."""
        mine_graph = {
            (0, 'A'): [((0, 'B'), 'instavel'), ((0, 'C'), 'normal')],
            (0, 'B'): [((0, 'A'), 'instavel'), ((0, 'D'), 'normal')],
            (0, 'C'): [((0, 'A'), 'normal'), ((0, 'D'), 'normal')],
            (0, 'D'): [((0, 'B'), 'normal'), ((0, 'C'), 'normal')],
        }
        env = MineEnvironment(mine_graph)
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'A'),
            target_location=(0, 'D'),
            environment=env,
            battery=30
        )
        solution = astar_search(problem)
        assert solution is not None
        # Caminho via C (custo 1+1=2) deve ser preferido a via B (custo 4+1=5)
        assert solution.path_cost == 2

    def test_battery_constraint(self, simple_mine):
        """Agente com bateria insuficiente não deve conseguir chegar."""
        env = simple_mine
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'A'),
            target_location=(0, 'C'),
            environment=env,
            battery=1  # Só dá para 1 passo, precisa de 2
        )
        solution = astar_search(problem)
        assert solution is None


# =============================================================================
# Testes do Agente
# =============================================================================

class TestRescueAgent:
    """Testes para o programa do agente."""

    def test_agent_creation(self, simple_mine):
        """Agente deve ser criado corretamente."""
        env = simple_mine
        robot = create_rescue_agent(env, search_algorithm='astar', battery=30)
        assert isinstance(robot, RescueRobot)
        assert robot.battery == 30
        assert robot.alive is True

    def test_agent_updates_state(self, simple_mine):
        """Agente deve atualizar estado com percepção."""
        env = simple_mine
        program = MineRescueAgentProgram(env, 'astar')
        percept = {
            'location': (0, 'A'),
            'adjacent_tunnels': [{'destination': (0, 'B'), 'type': 'normal', 'cost': 1}],
            'miners_here': [],
            'battery': 30,
            'step': 0,
            'rescued_miners': [],
            'all_miners': [],
        }
        state = program.update_state(None, percept)
        assert state['location'] == (0, 'A')
        assert state['battery'] == 30

    def test_agent_formulates_goal(self, simple_mine):
        """Agente deve formular objetivo corretamente."""
        env = simple_mine
        program = MineRescueAgentProgram(env, 'astar')
        program.state = {
            'location': (0, 'A'),
            'miners_here': [],
            'all_miners': [{'location': (0, 'C'), 'priority': 1, 'oxygen': 10}],
            'rescued_miners': [],
        }
        goal = program.formulate_goal(program.state)
        assert goal == (0, 'C')

    def test_agent_rescues_miner(self, simple_mine):
        """Agente deve conseguir resgatar minerador."""
        env = simple_mine
        miner = Miner(priority=1, oxygen=20)
        env.add_thing(miner, location=(0, 'C'))

        robot = create_rescue_agent(env, search_algorithm='astar', battery=30)
        env.add_thing(robot, location=(0, 'A'))

        # Executar simulação
        env.run(steps=20)

        assert miner.rescued is True

    def test_agent_prioritizes_urgent(self):
        """Agente deve priorizar minerador mais urgente."""
        mine_graph = {
            (0, 'A'): [((0, 'B'), 'normal'), ((0, 'C'), 'normal')],
            (0, 'B'): [((0, 'A'), 'normal')],
            (0, 'C'): [((0, 'A'), 'normal')],
        }
        env = MineEnvironment(mine_graph)
        env.max_steps = 30

        m1 = Miner(priority=1, oxygen=5)   # Urgente
        m2 = Miner(priority=3, oxygen=30)   # Menos urgente
        env.add_thing(m1, location=(0, 'B'))
        env.add_thing(m2, location=(0, 'C'))

        robot = create_rescue_agent(env, search_algorithm='astar', battery=30)
        env.add_thing(robot, location=(0, 'A'))

        # O agente deve ir primeiro para B (prioridade 1, menos oxigênio)
        percept = env.percept(robot)
        action = robot.program(percept)

        # Verifica se a primeira ação é mover para B (minerador urgente)
        assert action == ('move', (0, 'B'))

    def test_agent_with_different_algorithms(self, simple_mine):
        """Agente deve funcionar com diferentes algoritmos."""
        for algo in ['astar', 'ucs', 'greedy', 'bfs']:
            env = MineEnvironment({
                (0, 'A'): [((0, 'B'), 'normal')],
                (0, 'B'): [((0, 'A'), 'normal')],
            })
            env.max_steps = 20
            miner = Miner(priority=1, oxygen=15)
            env.add_thing(miner, location=(0, 'B'))
            robot = create_rescue_agent(env, search_algorithm=algo, battery=20)
            env.add_thing(robot, location=(0, 'A'))
            env.run(steps=10)
            assert miner.rescued is True, f"Falhou com algoritmo {algo}"

    def test_agent_stats(self, simple_mine):
        """Agente deve registrar estatísticas."""
        env = simple_mine
        env.max_steps = 20
        miner = Miner(priority=1, oxygen=20)
        env.add_thing(miner, location=(0, 'C'))
        robot = create_rescue_agent(env, search_algorithm='astar', battery=30)
        env.add_thing(robot, location=(0, 'A'))

        env.run(steps=15)

        stats = robot.program.get_stats()
        assert stats['searches_performed'] > 0
        assert stats['actions_executed'] > 0


# =============================================================================
# Testes de Integração
# =============================================================================

class TestIntegration:
    """Testes de integração completa."""

    def test_full_default_scenario(self):
        """Cenário padrão completo deve funcionar."""
        mine_graph, collapse_schedule = create_default_mine()
        env = MineEnvironment(mine_graph, collapse_schedule)
        env.max_steps = 60

        env.add_thing(Miner(priority=1, oxygen=15), location=(1, 'F'))
        env.add_thing(Miner(priority=2, oxygen=25), location=(2, 'I'))
        env.add_thing(Miner(priority=3, oxygen=30), location=(0, 'C'))

        robot = create_rescue_agent(env, search_algorithm='astar', battery=50)
        env.add_thing(robot, location=(0, 'A'))

        env.run(steps=60)

        rescued = [t for t in env.things
                   if isinstance(t, Miner) and t.rescued]
        assert len(rescued) >= 1  # Pelo menos 1 resgatado

    def test_full_complex_scenario(self):
        """Cenário complexo deve funcionar sem erros."""
        mine_graph, collapse_schedule = create_complex_mine()
        env = MineEnvironment(mine_graph, collapse_schedule)
        env.max_steps = 80

        env.add_thing(Miner(priority=1, oxygen=10), location=(2, 'H'))
        env.add_thing(Miner(priority=1, oxygen=12), location=(2, 'K'))
        env.add_thing(Miner(priority=2, oxygen=20), location=(1, 'F'))
        env.add_thing(Miner(priority=3, oxygen=35), location=(1, 'E'))

        robot = create_rescue_agent(env, search_algorithm='astar', battery=60)
        env.add_thing(robot, location=(0, 'A'))

        # Não deve lançar exceção
        env.run(steps=80)

    def test_collapse_forces_replan(self):
        """Colapso deve forçar o agente a replanar."""
        mine_graph = {
            (0, 'A'): [((0, 'B'), 'normal'), ((0, 'C'), 'normal')],
            (0, 'B'): [((0, 'A'), 'normal'), ((0, 'D'), 'normal')],
            (0, 'C'): [((0, 'A'), 'normal'), ((0, 'D'), 'normal')],
            (0, 'D'): [((0, 'B'), 'normal'), ((0, 'C'), 'normal')],
        }
        collapse_schedule = [
            ((0, 'A'), (0, 'B'), 1),  # Colapsa imediatamente
        ]
        env = MineEnvironment(mine_graph, collapse_schedule)
        env.max_steps = 20

        env.add_thing(Miner(priority=1, oxygen=15), location=(0, 'D'))
        robot = create_rescue_agent(env, search_algorithm='astar', battery=20)
        env.add_thing(robot, location=(0, 'A'))

        env.run(steps=15)

        # Minerador deve ser resgatado via rota alternativa (A→C→D)
        miners = [t for t in env.things if isinstance(t, Miner)]
        assert miners[0].rescued is True

    def test_performance_calculation(self, simple_mine):
        """Performance deve ser calculada corretamente."""
        env = simple_mine
        env.max_steps = 20
        miner = Miner(priority=1, oxygen=20)
        env.add_thing(miner, location=(0, 'C'))
        robot = create_rescue_agent(env, search_algorithm='astar', battery=30)
        env.add_thing(robot, location=(0, 'A'))

        env.run(steps=15)

        # Performance = +100 (resgate P1) - custo_caminho
        assert robot.performance > 0

    def test_render_during_simulation(self, default_mine):
        """Render deve funcionar durante toda a simulação."""
        env = default_mine
        env.add_thing(Miner(priority=1, oxygen=15), location=(1, 'F'))
        robot = create_rescue_agent(env, search_algorithm='astar', battery=50)
        env.add_thing(robot, location=(0, 'A'))

        for _ in range(5):
            if env.is_done():
                break
            env.step()
            env.render()  # Não deve lançar exceção


# =============================================================================
# Testes das Heurísticas
# =============================================================================

class TestHeuristics:
    """Testes para as heurísticas."""

    def test_heuristic_consistency(self):
        """Heurística deve ser consistente: h(n) <= c(n,a,n') + h(n')."""
        mine_graph = {
            (0, 'A'): [((0, 'B'), 'normal'), ((0, 'C'), 'estreito')],
            (0, 'B'): [((0, 'A'), 'normal'), ((0, 'D'), 'normal')],
            (0, 'C'): [((0, 'A'), 'estreito'), ((0, 'D'), 'normal')],
            (0, 'D'): [((0, 'B'), 'normal'), ((0, 'C'), 'normal')],
        }
        env = MineEnvironment(mine_graph)
        problem = SingleMinerRescueProblem(
            initial_location=(0, 'A'),
            target_location=(0, 'D'),
            environment=env,
            battery=30
        )

        from search import Node
        # Testar consistência para todos os nós
        for node_state in [(0, 'A'), (0, 'B'), (0, 'C'), (0, 'D')]:
            state = (node_state, 30, 0)
            node = Node(state=state)
            h_n = problem.h(node)

            for action in problem.actions(state):
                new_state = problem.result(state, action)
                child_node = Node(state=new_state)
                h_n_prime = problem.h(child_node)
                cost = problem.path_cost(0, state, action, new_state)

                # h(n) <= c(n,a,n') + h(n')
                assert h_n <= cost + h_n_prime, \
                    f"Inconsistente em {node_state}: h={h_n}, c={cost}, h'={h_n_prime}"

    def test_mst_heuristic(self, simple_mine):
        """Heurística MST deve ser admissível."""
        env = simple_mine
        env.add_thing(Miner(priority=1, oxygen=20), location=(0, 'B'))
        env.add_thing(Miner(priority=2, oxygen=20), location=(0, 'C'))

        problem = MineRescueMultiProblem(
            initial_location=(0, 'A'),
            miner_targets=[(0, 'B'), (0, 'C')],
            environment=env,
            battery=30
        )

        from search import Node
        node = Node(state=problem.initial)
        h_value = problem.h(node)

        # MST de {A, B, C}: A-B (1) + B-C (1) = 2
        # Custo real mínimo: A→B→C = 2 (+ 2 resgates = 4)
        # h deve ser <= custo real
        assert h_value <= 4


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
