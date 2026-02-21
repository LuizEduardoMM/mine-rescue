"""
Main - Agente de Resgate em Mina Subterrânea
=============================================

Ponto de entrada principal do projeto.
Demonstra o agente em execução com diferentes cenários e algoritmos.

Uso:
    python main.py                  # Executa cenário padrão com A*
    python main.py --scenario complex  # Cenário complexo
    python main.py --algorithm ucs  # Usar Uniform Cost Search
    python main.py --compare        # Comparar todos os algoritmos
"""

import sys
import os
import argparse
import time

# Configurar paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'aima-python'))
sys.path.insert(0, os.path.dirname(__file__))

from env.mine_environment import (MineEnvironment, Miner, RescueRobot,
                                   create_default_mine, create_complex_mine)
from agents_pkg.rescue_agent import (create_rescue_agent, create_multi_rescue_agent,
                                      MineRescueAgentProgram)
from problems.mine_problem import (MineRescueProblem, SingleMinerRescueProblem,
                                    MineRescueMultiProblem)


def setup_default_scenario():
    """
    Configura o cenário padrão com 3 mineradores.

    Mina com 3 níveis:
    - Nível 0: Entrada (A), sala B, sala C
    - Nível 1: sala D (elevador), E (inundado), F
    - Nível 2: sala G (elevador), H (instável), I

    Mineradores:
    - Minerador P1 (alta prioridade, O2=15) na sala F (nível 1)
    - Minerador P2 (média prioridade, O2=25) na sala I (nível 2)
    - Minerador P3 (baixa prioridade, O2=30) na sala C (nível 0)

    Colapsos:
    - Túnel D-E colapsa no passo 12
    - Túnel G-H colapsa no passo 18
    """
    mine_graph, collapse_schedule = create_default_mine()
    env = MineEnvironment(mine_graph, collapse_schedule)
    env.max_steps = 60

    # Adicionar mineradores
    miner1 = Miner(priority=1, oxygen=15)
    miner2 = Miner(priority=2, oxygen=25)
    miner3 = Miner(priority=3, oxygen=30)

    env.add_thing(miner1, location=(1, 'F'))
    env.add_thing(miner2, location=(2, 'I'))
    env.add_thing(miner3, location=(0, 'C'))

    return env


def setup_complex_scenario():
    """
    Configura o cenário complexo com mais mineradores e rotas alternativas.

    Mineradores:
    - P1 (O2=10) na sala H (nível 2) — URGENTE
    - P1 (O2=12) na sala K (nível 2) — URGENTE
    - P2 (O2=20) na sala F (nível 1)
    - P3 (O2=35) na sala E (nível 1)
    """
    mine_graph, collapse_schedule = create_complex_mine()
    env = MineEnvironment(mine_graph, collapse_schedule)
    env.max_steps = 80

    env.add_thing(Miner(priority=1, oxygen=10), location=(2, 'H'))
    env.add_thing(Miner(priority=1, oxygen=12), location=(2, 'K'))
    env.add_thing(Miner(priority=2, oxygen=20), location=(1, 'F'))
    env.add_thing(Miner(priority=3, oxygen=35), location=(1, 'E'))

    return env


def run_simulation(env, algorithm='astar', battery=50, verbose=True):
    """
    Executa uma simulação completa.

    Args:
        env: MineEnvironment configurado
        algorithm: Algoritmo de busca ('astar', 'ucs', 'greedy', 'bfs')
        battery: Bateria inicial do robô
        verbose: Se True, exibe render() a cada passo

    Returns:
        dict com resultados da simulação
    """
    # Criar agente
    robot = create_rescue_agent(env, search_algorithm=algorithm, battery=battery)
    env.add_thing(robot, location=(0, 'A'))

    if verbose:
        print(f"\n{'#' * 60}")
        print(f"  SIMULAÇÃO - Algoritmo: {algorithm.upper()}")
        print(f"  Bateria: {battery} | Max Steps: {env.max_steps}")
        print(f"{'#' * 60}")
        env.render()

    # Executar simulação passo a passo
    start_time = time.time()

    while not env.is_done():
        env.step()
        if verbose:
            env.render()
            time.sleep(0.1)  # Pausa para visualização

    elapsed = time.time() - start_time

    # Coletar resultados
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
    """
    Compara todos os algoritmos de busca no mesmo cenário.
    """
    algorithms = ['astar', 'ucs', 'greedy', 'bfs']

    print(f"\n{'#' * 70}")
    print(f"  COMPARAÇÃO DE ALGORITMOS - Cenário: {scenario}")
    print(f"{'#' * 70}")

    all_results = []

    for algo in algorithms:
        # Recriar ambiente para cada algoritmo (estado limpo)
        if scenario == 'complex':
            env = setup_complex_scenario()
        else:
            env = setup_default_scenario()

        print(f"\n  >>> Executando {algo.upper()}...")
        results = run_simulation(env, algorithm=algo, battery=50, verbose=False)
        all_results.append(results)

    # Tabela comparativa
    print(f"\n{'=' * 70}")
    print(f"  {'Algoritmo':<12} {'Resgatados':<12} {'Mortos':<8} "
          f"{'Perf.':<10} {'Bateria':<10} {'Passos':<8} {'Buscas':<8}")
    print(f"  {'-' * 64}")

    for r in all_results:
        stats = r['agent_stats']
        print(f"  {r['algorithm'].upper():<12} {r['rescued']:<12} {r['dead']:<8} "
              f"{r['performance']:<10} {r['battery_remaining']:<10} "
              f"{r['steps']:<8} {stats.get('searches_performed', 'N/A'):<8}")

    print(f"{'=' * 70}")

    # Identificar melhor
    best = max(all_results, key=lambda r: (r['rescued'], r['performance']))
    print(f"\n  Melhor algoritmo: {best['algorithm'].upper()}")
    print(f"     Resgatados: {best['rescued']} | Performance: {best['performance']}")

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
