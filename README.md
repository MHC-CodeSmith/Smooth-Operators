# RRRP - Planejamento, Simulação e Dinâmica Simbólica

Este repositório reúne três scripts Python para geração de trajetórias, simulação de controle e análise simbólica de um robô 4-DOF do tipo RRRP (Rotação–Rotação–Rotação–Prismática). Cada ferramenta atua em uma etapa distinta do fluxo de trabalho, desde a definição geométrica até a extração das equações de torque.

---

## Índice

* [Visão Geral](#visão-geral)
* [Estrutura do Repositório](#estrutura-do-repositório)
* [Pré-requisitos](#pré-requisitos)
* [Instalação](#instalação)
* [Fluxo de Trabalho](#fluxo-de-trabalho)
* [Descrição Detalhada dos Scripts](#descrição-detalhada-dos-scripts)

  * [1. kinematics\_planner.py](#1-kinematics_plannerpy)
  * [2. controller\_simulation.py](#2-controller_simulationpy)
  * [3. gerador\_simbolico\_puro.py](#3-gerador_simbolico_puropy)
* [Exemplos de Execução](#exemplos-de-execução)
* [Interpretando Resultados](#interpretando-resultados)
* [Configurações e Parâmetros](#configurações-e-parâmetros)
* [Contribuições](#contribuições)
* [Contato](#contato)

---

## Visão Geral

1. **Modelagem Cinemática**: com `kinematics_planner.py`, define-se o modelo DH e gera-se, por aproximação cartesiana, a trajetória desejada das juntas.
2. **Simulação Dinâmica e Controle**: em `controller_simulation.py`, implementa-se um controlador PID completo, simula-se numericamente a dinâmica (inércia, Coriolis, gravidade), e avalia-se o desempenho via gráficos e métricas.
3. **Dinâmica Simbólica**: com `gerador_simbolico_puro.py`, obtém-se as matrizes M(q), C(q,q̇), G(q) e equações de torque τ = M·q̈ + C·q̇ + G em forma simbólica, permitindo análise teórica e validação de parâmetros.

---

## Estrutura do Repositório

```text
├── data/
│   └── trajetoria_desejada.npy    # Saída do planejador de cinemática
│   └── resultados_simulacao/      # (opcional) logs e CSVs de saída
├── kinematics_planner.py         # Geração de trajetória
├── controller_simulation.py      # Simulação de controle dinâmico
├── gerador_simbolico_puro.py     # Derivação simbólica da dinâmica
├── requirements.txt              # Dependências Python
└── README.md                     # Documentação
```

---

## Pré-requisitos

* Python 3.8+
* `pip install -r requirements.txt`

**`requirements.txt`**

```
numpy
sympy
matplotlib
roboticstoolbox-python
spatialmath-python
scipy
pandas
scikit-learn
```

---

## Instalação

1. Clone o repositório:

   ```bash
   ```

git clone [https://github.com/seu-usuario/rrrp-simulation.git](https://github.com/seu-usuario/rrrp-simulation.git)
cd rrrp-simulation

````
2. Crie e ative um ambiente virtual (opcional, mas recomendado):
```bash
python -m venv venv
source venv/bin/activate  # Linux/macOS
venv\Scripts\activate     # Windows
````

3. Instale as dependências:

   ```bash
   ```

pip install -r requirements.txt

````

---

## Fluxo de Trabalho

1. **Planejamento Cinêmatico**: define o modelo e gera a trajetória de referência.
2. **Simulação de Controle**: executa a simulação PID usando como entrada a trajetória gerada.
3. **Análise Simbólica**: extrai e verifica as equações de dinâmica para pesquisa ou publicação.

---

## Descrição Detalhada dos Scripts

### 1. kinematics_planner.py

- **Objetivo**: gerar `trajetoria_desejada.npy`, uma matriz Q com dimensões (N×4), onde cada linha representa [θ₁, θ₂, θ₃, d₄] desejados.
- **Principais Funções**:
- `create_rrrp_robot_from_dh()`: monta o modelo DH para validação (se Toolbox disponível).
- `generate_trajectory(N)`:
 1. Define um deslocamento linear no TCP ao longo do eixo X usando `linspace`.
 2. Calcula q₂ via arcsin para manter trajetória curva, q₃ como complemento π/2−q₂.
 3. Ajusta q₄ para compensar a extensão prismática necessária.
 4. Retorna Q = [zeros, q₂, q₃, q₄].
- `main()`: chama gerar trajetória e salva o array NumPy.

### 2. controller_simulation.py

- **Objetivo**: simular o comportamento dinâmico do robô sob controle PID e avaliar precisão de seguimento.
- **Processo Interno**:
1. **Carregamento da Trajetória**: lê `trajetoria_desejada.npy` e fixa θ₁=0.
2. **Cálculo de Derivadas**: obtém velocidades (`gradient`) e acelerações desejadas.
3. **Ganho PID**: matrizes Kp, Kd, Ki definem o desempenho (overshoot, tempo de assentamento).
4. **Loop de Simulação** (passo a passo):
  - Calcula erro, erro derivativo e integral.
  - Compensa aceleração desejada: `acc_des = Kp·erro + Kd·derr + Ki·∫erro`.
  - Usa M(q), C(q,qd), G(q) do modelo para calcular torque: `τ = M·acc_des + C·qd + G`.
  - Integra numericamente qdd → qd → q.
5. **Animação (opcional)**: exibe modelo 3D via Robotics Toolbox.
6. **Plotagem**: gera gráficos de:
  - Posições desejada vs. real.
  - Erro de seguimento.
  - Torque aplicado.
  - Análise avançada: overshoot, erro em regime, tempo de estabilização, histograma e estatísticas.

### 3. gerador_simbolico_puro.py

- **Objetivo**: derivar simbolicamente toda a dinâmica para validação teórica.
- **Passos Principais**:
1. Definição de variáveis simbólicas (q(t), q̇, q̈, parâmetros físicos e tensores de inércia).
2. Cálculo de transformações DH (matrizes T01, T12, T23, T34).
3. Velocidades lineares e angulares de cada elo.
4. Energia cinética (translacional + rotacional) e potencial gravitacional.
5. Derivação das matrizes M(q), G(q) e montagem de C(q,q̇) via símbolos de Christoffel.
6. Simplificação trigonométrica e montagem do vetor τ = M·q̈ + C·q̇ + G.
7. Substituição θ₁=0, q̇₁=0, q̈₁=0 para exibir as fórmulas finais.

- **Saída**: matrizes e expressões no terminal; ideal para conferência com trabalhos acadêmicos.

---

## Exemplos de Execução

```bash
# 1) Planejar cinemática
env/bin/python kinematics_planner.py
# gera: data/trajetoria_desejada.npy

# 2) Simular controle
env/bin/python controller_simulation.py
# exibe animação, salva gráficos e imprime métricas

# 3) Gerar dinâmica simbólica
env/bin/python gerador_simbolico_puro.py
# imprime passo a passo e fórmulas finais
````

---

## Interpretando Resultados

* **trajetoria\_desejada.npy**: array N×4. Use NumPy para carregar e inspecionar:

  ```python
  import numpy as np
  Q = np.load('data/trajetoria_desejada.npy')
  print(Q[:5])  # primeiros 5 pontos
  ```
* **Gráficos de Simulação**:

  * Deslocamento das juntas: avalie seguimento (curvas desejada vs. real).
  * Erro de seguimento: identifique oscilações e estabilidade.
  * Torque: analise picos de torque e possíveis saturações.
* **Métricas** (controller\_simulation): overshoot, erro em estado estacionário e tempo de assentamento.
* **Expressões Simbólicas**: confira coerência com literatura e use em publicações.

---

## Configurações e Parâmetros

* **Número de Pontos (N)**: ajuste em `generate_trajectory(N)` para resolução da trajetória.
* **Ganho PID**: valores padrão em `controller_simulation.py` servem como ponto de partida; re-tune conforme carga e resposta desejada.
* **Limites de Junta Prismática**: \[0, 0.105] m — altere em ambos os scripts se o robô físico tiver alcance diferente.

---

## Contribuições

Pull requests são bem-vindos! Para melhorias de estabilidade, novos controladores ou suporte a outros robôs, abra uma issue ou PR.

---

## Contato

**Matheus Hipolito Carvalho**
E-mail: [matheus.hipolito@usp.br](mailto:matheus.hipolito@usp.br)
GitHub: [github.com/seu-usuario](MHC_CodeSmith)
