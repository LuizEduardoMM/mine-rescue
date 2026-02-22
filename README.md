# Agente de Resgate em Mina Subterrânea

## Disciplina: Inteligência Artificial

**Referências:**
- Russell & Norvig – *Artificial Intelligence: A Modern Approach* (AIMA)
- Repositório oficial: https://github.com/aimacode/aima-python

---

## Descrição do Problema

Um agente robô precisa resgatar mineradores presos em diferentes pontos de uma mina subterrânea com múltiplos níveis. A mina possui:

- **Túneis com diferentes custos**: normal (1), estreito (2), inundado (3), instável (4)
- **Níveis de profundidade** conectados por elevadores (custo 2)
- **Recursos limitados**: bateria do robô, oxigênio das vítimas
- **Áreas em colapso progressivo**: algumas rotas ficam inacessíveis com o tempo
- **Múltiplas vítimas** com diferentes prioridades de resgate

---

## Especificação Formal do Problema (AIMA)

### Representação dos Estados
**Estado** = `(posição_agente, frozenset(mineradores_resgatados), bateria, step)`
- `posição_agente`: tupla `(nível, identificador)` ex: `(0, 'A')`
- `mineradores_resgatados`: conjunto imutável de localizações já resgatadas
- `bateria`: energia restante do robô
- `step`: passo atual (tempo discreto)

**Implementação**: `problems/mine_problem.py` — classes `MineRescueProblem`, `SingleMinerRescueProblem`

### Estado Inicial
`((0, 'A'), frozenset(), 50, 0)` — agente na entrada, nenhum resgatado, bateria cheia.

**Implementação**: `__init__` de cada subclasse de `Problem`

### Conjunto de Ações
- `('move', destino)`: Mover para nó adjacente via túnel não colapsado
- `('rescue',)`: Resgatar minerador(es) na posição atual

**Implementação**: método `actions(state)` nas subclasses de `Problem`

### Modelo de Transição — `result(s, a)`
- **move**: atualiza posição, reduz bateria pelo custo do túnel, incrementa step
- **rescue**: adiciona localização aos resgatados, reduz bateria em 1

**Implementação**: método `result(state, action)` nas subclasses de `Problem`

### Teste de Objetivo — `goal_test(s)`
Todos os mineradores-alvo foram resgatados: `target_set ⊆ rescued_set`

**Implementação**: método `goal_test(state)` nas subclasses de `Problem`

### Custo de Caminho — `path_cost`
Soma dos custos reais dos túneis percorridos + custos de resgate.

| Tipo de Túnel | Custo |
|---------------|-------|
| Normal        | 1     |
| Estreito      | 2     |
| Inundado      | 3     |
| Instável      | 4     |
| Elevador      | 2     |

**Implementação**: método `path_cost(c, state1, action, state2)` nas subclasses de `Problem`

---

## Classificação do Ambiente (AIMA)

| Critério | Classificação | Justificativa |
|----------|---------------|---------------|
| **Observabilidade** | **Parcialmente observável** | O agente só percebe túneis adjacentes, mineradores na posição atual e seu nível de bateria. Não tem visão global. |
| **Determinismo** | **Determinístico com mudanças exógenas** | As ações do agente têm resultado previsível, mas colapsos ocorrem independentemente (baseados no tempo). |
| **Dinâmica** | **Dinâmico** | Áreas entram em colapso progressivo; oxigênio das vítimas diminui a cada passo. |
| **Discretude** | **Discreto** | Posições são nós do grafo, ações são discretas, tempo avança em passos. |
| **Agentes** | **Agente único** | Um único robô de resgate opera na mina. |

---

## Arquitetura Ambiente – Agente – Programa de Agente

### Ambiente (`env/mine_environment.py`)
- **Classe**: `MineEnvironment(Environment)` — herda de `agents.Environment` do aima-python
- Mantém o grafo da mina, posições, colapsos, contagem de tempo
- Fornece percepções via `percept(agent)`
- Executa ações via `execute_action(agent, action)`
- Mudanças exógenas via `exogenous_change()` (colapsos, redução de O2)
- Renderização via `render()`

### Agente (`env/mine_environment.py`)
- **Classe**: `RescueRobot(Agent)` — herda de `agents.Agent` do aima-python
- Possui bateria, lista de resgatados, estado vital

### Programa de Agente (`agents/rescue_agent.py`)
- **Classe**: `MineRescueAgentProgram(SimpleProblemSolvingAgentProgram)` — herda da Figure 3.1 do AIMA
- Recebe percepções → atualiza estado → formula objetivo → formula problema → busca → executa ação
- **NÃO é** o algoritmo de busca — ele *usa* algoritmos de busca internamente
- Implementa replanejamento quando colapsos invalidam o plano atual

---

## Algoritmos de Busca

### Utilizados

| Algoritmo | Uso | Justificativa |
|-----------|-----|---------------|
| **A\*** | Principal | Ótimo e completo com heurística admissível; melhor equilíbrio entre qualidade e eficiência |
| **Uniform Cost Search** | Fallback | Ótimo sem heurística; usado quando A* falha |
| **Greedy Best-First** | Emergências | Mais rápido que A*; não garante otimalidade mas útil para decisões rápidas |
| **BFS** | Verificação | Completo; útil para verificar acessibilidade |

### NÃO Utilizados

| Algoritmo | Justificativa |
|-----------|---------------|
| **DFS** | Não garante solução ótima; pode explorar caminhos muito longos |
| **Depth-Limited** | Difícil definir limite adequado em mina com profundidade variável |
| **Iterative Deepening** | Re-expande muitos nós; ineficiente para grafos com custos variáveis |
| **Hill Climbing** | Não é completo; pode ficar preso em ótimos locais |
| **Simulated Annealing** | Para otimização contínua, não planejamento de caminho |
| **Genetic Algorithm** | Não adequado para planejamento sequencial de ações |

---

## Heurísticas

### Heurística Principal: Distância Mínima no Grafo

**Definição**: `h(n) = distância mínima pré-calculada (Dijkstra) da posição atual até o minerador-alvo mais distante`

**Intuição**: O agente precisa, no mínimo, percorrer a distância até o minerador mais distante não resgatado.

**Admissibilidade**: Usa distâncias reais do grafo (Dijkstra). Nunca superestima porque o custo real é >= distância mínima.

**Consistência**: `h(n) <= c(n,a,n') + h(n')` — a distância mínima diminui em no máximo o custo da aresta percorrida.

### Heurística MST (Variante Multi-objetivo)

**Definição**: `h(n) = custo da Árvore Geradora Mínima sobre {posição_atual} + {mineradores_restantes}`

**Admissibilidade**: MST é limite inferior do tour ótimo (TSP).

**Implementação**: `MineRescueMultiProblem.h(node)` em `problems/mine_problem.py`

---

## Estrutura do Projeto

```
mine_rescue/
├── __init__.py
├── main.py                          # Ponto de entrada principal
├── README.md                        # Este arquivo
├── env/
│   ├── __init__.py
│   └── mine_environment.py          # Ambiente (MineEnvironment, Miner, RescueRobot)
├── agents_pkg/
│   ├── __init__.py
│   └── rescue_agent.py              # Programa do agente (MineRescueAgentProgram)
├── problems/
│   ├── __init__.py
│   └── mine_problem.py              # Problemas de busca (subclasses de Problem)
└── tests/
    ├── __init__.py
    └── test_mine_rescue.py           # Testes automatizados (pytest)
```

---

## Instruções de Execução

### 1. Clonar o repositório (com submodule)
```bash
git clone --recurse-submodules https://github.com/LuizEduardoMM/mine-rescue-agent.git
cd mine-rescue-agent
```
> Se já clonou sem `--recurse-submodules`, execute:
> ```bash
> git submodule update --init
> ```

### 2. Instalar dependências
```bash
pip install pytest numpy
```

### 3. Executar a simulação

**Simulação padrão com A\* (com render passo a passo):**
```bash
cd mine_rescue
python main.py
```

**Cenário complexo (mais mineradores e rotas):**
```bash
python main.py --scenario complex
```

**Escolher algoritmo de busca:**
```bash
python main.py --algorithm astar    # A* (padrão)
python main.py --algorithm ucs      # Uniform Cost Search
python main.py --algorithm greedy   # Greedy Best-First
python main.py --algorithm bfs      # Breadth-First Search
```

**Comparar todos os algoritmos lado a lado:**
```bash
python main.py --compare
python main.py --compare --scenario complex
```

**Modo silencioso (sem render, só resultado final):**
```bash
python main.py --quiet
python main.py --scenario complex --algorithm ucs --quiet
```

### 4. Executar os testes automatizados
```bash
cd mine_rescue
pytest tests/test_mine_rescue.py -v
```

---

## Testes

Os testes estão em `tests/test_mine_rescue.py` e cobrem:

1. **TestMineEnvironment** — Criação do ambiente, adjacências, túneis, percepções, ações, colapsos, render
2. **TestMineRescueProblem** — Estado inicial, ações, transições, goal_test, path_cost, heurísticas, soluções com A*/UCS/BFS
3. **TestRescueAgent** — Criação do agente, atualização de estado, formulação de objetivos, priorização, múltiplos algoritmos
4. **TestIntegration** — Cenários completos, replanejamento por colapso, cálculo de performance
5. **TestHeuristics** — Admissibilidade e consistência das heurísticas

---

## Mapeamento Código vs Especificação

| Especificação AIMA | Arquivo | Classe/Método |
|---------------------|---------|---------------|
| Estado | `problems/mine_problem.py` | Tupla `(pos, rescued, battery, step)` |
| Estado inicial | `problems/mine_problem.py` | `__init__` → `initial_state` |
| Ações | `problems/mine_problem.py` | `actions(state)` |
| Modelo de transição | `problems/mine_problem.py` | `result(state, action)` |
| Teste de objetivo | `problems/mine_problem.py` | `goal_test(state)` |
| Custo de caminho | `problems/mine_problem.py` | `path_cost(c, s1, a, s2)` |
| Heurística | `problems/mine_problem.py` | `h(node)` |
| Ambiente | `env/mine_environment.py` | `MineEnvironment(Environment)` |
| Percepção | `env/mine_environment.py` | `percept(agent)` |
| Execução de ação | `env/mine_environment.py` | `execute_action(agent, action)` |
| Mudança exógena | `env/mine_environment.py` | `exogenous_change()` |
| Renderização | `env/mine_environment.py` | `render()` |
| Agente | `env/mine_environment.py` | `RescueRobot(Agent)` |
| Programa de agente | `agents_pkg/rescue_agent.py` | `MineRescueAgentProgram(SimpleProblemSolvingAgentProgram)` |
| Formulação objetivo | `agents_pkg/rescue_agent.py` | `formulate_goal(state)` |
| Formulação problema | `agents_pkg/rescue_agent.py` | `formulate_problem(state, goal)` |
| Busca | `agents_pkg/rescue_agent.py` | `search(problem)` → `astar_search()` |
