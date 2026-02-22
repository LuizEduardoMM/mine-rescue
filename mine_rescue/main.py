"""Ponto de entrada principal. Executa simulação ou compara algoritmos."""

import sys
import os
import argparse
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'aima-python'))
sys.path.insert(0, os.path.dirname(__file__))

from env.mine_environment import (MineEnvironment, Miner, RescueRobot,
                                   create_default_mine, create_complex_mine)
from agents_pkg.rescue_agent import (create_rescue_agent, create_multi_rescue_agent,
                                      MineRescueAgentProgram)
from problems.mine_problem import (MineRescueProblem, SingleMinerRescueProblem,
                                    MineRescueMultiProblem)


def setup_default_scenario():
    """Cenário padrão: 3 mineradores, 3 níveis."""
    mine_graph, collapse_schedule = create_default_mine()
    env = MineEnvironment(mine_graph, collapse_schedule)
    env.max_steps = 60

    miner1 = Miner(priority=1, oxygen=15)
    miner2 = Miner(priority=2, oxygen=25)
    miner3 = Miner(priority=3, oxygen=30)

    env.add_thing(miner1, location=(1, 'F'))
    env.add_thing(miner2, location=(2, 'I'))
    env.add_thing(miner3, location=(0, 'C'))

    return env


def setup_complex_scenario():
    """Cenário complexo: 4 mineradores, rotas alternativas com custos diferentes."""
    mine_graph, collapse_schedule = create_complex_mine()
    env = MineEnvironment(mine_graph, collapse_schedule)
    env.max_steps = 80

    env.add_thing(Miner(priority=1, oxygen=6),  location=(1, 'F'))
    env.add_thing(Miner(priority=1, oxygen=12), location=(2, 'H'))
    env.add_thing(Miner(priority=2, oxygen=20), location=(2, 'K'))
    env.add_thing(Miner(priority=3, oxygen=35), location=(1, 'E'))

    return env


def run_simulation(env, algorithm='astar', battery=50, verbose=True):
    robot = create_rescue_agent(env, search_algorithm=algorithm, battery=battery)
    env.add_thing(robot, location=(0, 'A'))

    if verbose:
        print(f"\n{'#' * 60}")
        print(f"  SIMULAÇÃO - Algoritmo: {algorithm.upper()}")
        print(f"  Bateria: {battery} | Max Steps: {env.max_steps}")
        print(f"{'#' * 60}")
        env.render()

    start_time = time.time()

    while not env.is_done():
        env.step()
        if verbose:
            env.render()
            time.sleep(0.1)  # Pausa para visualização

    elapsed = time.time() - start_time

    rescued = [t for t in env.things if isinstance(t, Miner) and t.rescued]
    dead = [t for t in env.things
            if isinstance(t, Miner) and not t.alive and not t.rescued]
    alive_not_rescued = [t for t in env.things
                         if isinstance(t, Miner) and t.alive and not t.rescued]

    agent_stats = robot.program.get_stats() if hasattr(robot.program, 'get_stats') else {}

    results = {
        'algorithm': algorithm,
        'rescued': len(rescued),
        'dead': len(dead),
        'alive_not_rescued': len(alive_not_rescued),
        'performance': robot.performance,
        'battery_remaining': robot.battery,
        'steps': env.step_count,
        'time_elapsed': elapsed,
        'agent_alive': robot.alive,
        'agent_stats': agent_stats,
    }

    if verbose:
        print(f"\n{'=' * 60}")
        print(f"  RESULTADO FINAL - {algorithm.upper()}")
        print(f"{'=' * 60}")
        print(f"  Mineradores resgatados: {results['rescued']}")
        print(f"  Mineradores falecidos:  {results['dead']}")
        print(f"  Não resgatados (vivos): {results['alive_not_rescued']}")
        print(f"  Performance do agente:  {results['performance']}")
        print(f"  Bateria restante:       {results['battery_remaining']}")
        print(f"  Passos executados:      {results['steps']}")
        print(f"  Tempo de execução:      {results['time_elapsed']:.3f}s")
        print(f"  Agente vivo:            {results['agent_alive']}")
        print(f"  Buscas realizadas:      {agent_stats.get('searches_performed', 'N/A')}")
        print(f"  Replanejamentos:        {agent_stats.get('replans', 'N/A')}")
        print(f"{'=' * 60}")

    return results


def compare_algorithms(scenario='default'):
    algorithms = ['astar', 'ucs', 'greedy', 'bfs']

    print(f"\n{'#' * 70}")
    print(f"  COMPARAÇÃO DE ALGORITMOS - Cenário: {scenario}")
    print(f"{'#' * 70}")

    all_results = []

    for algo in algorithms:
        if scenario == 'complex':
            env = setup_complex_scenario()
        else:
            env = setup_default_scenario()

        print(f"\n  >>> Executando {algo.upper()}...")
        results = run_simulation(env, algorithm=algo, battery=50, verbose=False)
        all_results.append(results)

    # Tabela comparativa
    battery_initial = 50
    print(f"\n{'=' * 80}")
    print(f"  {'Algoritmo':<12} {'Resgatados':<12} {'Mortos':<8} "
          f"{'Perf.':<10} {'Bat.Rest.':<10} {'Custo':<8} {'Passos':<8} {'NósExp.':<10}")
    print(f"  {'-' * 76}")

    for r in all_results:
        stats = r['agent_stats']
        path_cost = battery_initial - r['battery_remaining']
        nodes_exp = stats.get('nodes_expanded', 'N/A')
        print(f"  {r['algorithm'].upper():<12} {r['rescued']:<12} {r['dead']:<8} "
              f"{r['performance']:<10} {r['battery_remaining']:<10} "
              f"{path_cost:<8} {r['steps']:<8} {nodes_exp:<10}")

    print(f"{'=' * 80}")
    print(f"\n  Legenda:")
    print(f"  Custo    = bateria consumida (menor = caminho mais eficiente)")
    print(f"  NósExp.  = nós expandidos durante as buscas (menor = algoritmo mais eficiente)")
    print(f"  A*/UCS   preferem custo mínimo; BFS prefere menos saltos (pode custar mais)")
    print(f"  Greedy   segue a heurística; não garante otimalidade")

    # Identificar melhor
    best_perf = max(all_results, key=lambda r: (r['rescued'], r['performance']))
    least_cost = min(all_results, key=lambda r: r['battery_remaining'] and
                     (50 - r['battery_remaining']))
    least_nodes = min(all_results,
                      key=lambda r: r['agent_stats'].get('nodes_expanded', 9999))

    print(f"\n  Análise de tradeoffs:")
    print(f"  Melhor performance  : {best_perf['algorithm'].upper()}"
          f" (resgatados={best_perf['rescued']}, perf={best_perf['performance']})")
    print(f"  Menor custo (bateria): {min(all_results, key=lambda r: 50-r['battery_remaining'])['algorithm'].upper()}"
          f" (custo={50-min(all_results, key=lambda r: 50-r['battery_remaining'])['battery_remaining']})")
    print(f"  Menos nós expandidos: {least_nodes['algorithm'].upper()}"
          f" (nós={least_nodes['agent_stats'].get('nodes_expanded', 'N/A')})")

    return all_results


def main():
    parser = argparse.ArgumentParser(
        description='Agente de Resgate em Mina Subterrânea')
    parser.add_argument('--scenario', choices=['default', 'complex'],
                        default='default',
                        help='Cenário da mina (default ou complex)')
    parser.add_argument('--algorithm', choices=['astar', 'ucs', 'greedy', 'bfs'],
                        default='astar',
                        help='Algoritmo de busca')
    parser.add_argument('--battery', type=int, default=50,
                        help='Bateria inicial do robô')
    parser.add_argument('--compare', action='store_true',
                        help='Comparar todos os algoritmos')
    parser.add_argument('--quiet', action='store_true',
                        help='Modo silencioso (sem render)')

    args = parser.parse_args()

    if args.compare:
        compare_algorithms(args.scenario)
    else:
        if args.scenario == 'complex':
            env = setup_complex_scenario()
        else:
            env = setup_default_scenario()

        run_simulation(env, algorithm=args.algorithm,
                       battery=args.battery, verbose=not args.quiet)


if __name__ == '__main__':
    main()
