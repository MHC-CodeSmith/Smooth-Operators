# Simulação e Análise Dinâmica de Robô RRRP
![Python](https://img.shields.io/badge/Python-3.8%2B-blue?style=for-the-badge&logo=python)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Ativo-brightgreen?style=for-the-badge)

Este repositório oferece um conjunto de ferramentas em Python para o ciclo completo de análise de um robô manipulador 4-DOF do tipo **RRRP** (Rotacional-Rotacional-Rotacional-Prismático). O projeto é ideal para estudantes, pesquisadores e entusiastas da robótica que desejam explorar desde o planejamento da trajetória no espaço cartesiano até a derivação simbólica das equações de movimento e a simulação de controle dinâmico.

![Animação do Robô RRRP](https://i.imgur.com/your_animation.gif) 
> **Nota:** Recomenda-se substituir o link acima por um GIF gerado a partir da animação do `controller_simulation.py` para uma demonstração visual impactante.

---

## 📜 Índice

* [Visão Geral do Projeto](#-visão-geral-do-projeto)
* [Estrutura do Repositório](#-estrutura-do-repositório)
* [Configuração do Ambiente](#-configuração-do-ambiente)
* [Fluxo de Trabalho em 3 Passos](#-fluxo-de-trabalho-em-3-passos)
* [Descrição Detalhada dos Scripts](#-descrição-detalhada-dos-scripts)
  * [1. `kinematics_planner.py`](#1-kinematics_plannerpy)
  * [2. `controller_simulation.py`](#2-controller_simulationpy)
  * [3. `gerador_simbolico_puro.py`](#3-gerador_simbolico_puropy)
* [Como Executar](#-como-executar)
* [Interpretando os Resultados](#-interpretando-os-resultados)
* [Licença](#-licença)
* [Contato](#-contato)

---

## 🔭 Visão Geral do Projeto

Este projeto está dividido em três ferramentas principais, cada uma responsável por uma etapa crucial da análise robótica:

| Ferramenta | Objetivo |
| :--- | :--- |
| 🗺️ **Planejamento Cinemático** | `kinematics_planner.py`: Define uma trajetória suave para o efetuador final (TCP) no espaço cartesiano e, através da cinemática inversa, gera o arquivo com a sequência de posições desejadas para cada junta (`trajetoria_desejada.npy`). |
| ⚙️ **Simulação e Controle** | `controller_simulation.py`: Utiliza a trajetória gerada como entrada para um sistema de controle dinâmico. Simula o comportamento real do robô sob a ação de um controlador PID, considerando as forças de inércia, Coriolis e gravidade. Oferece uma rica análise de performance com gráficos, animação 3D e métricas estatísticas. |
| 🔬 **Análise Simbólica** | `gerador_simbolico_puro.py`: Deriva matematicamente as equações completas da dinâmica do robô usando álgebra simbólica. Gera as matrizes de Inércia `M(q)`, Coriolis `C(q, q̇)` e o vetor de Gravidade `G(q)`, essenciais para validação teórica, publicações acadêmicas e desenvolvimento de controladores avançados. |

---

## 🗂️ Estrutura do Repositório

```
rrrp-simulation/
│
├── data/
│   └── trajetoria_desejada.npy    # Arquivo de saída do planejador (Entrada da simulação)
│
├── docs/
│   └── rrrp_animation.gif         # (Opcional) Local sugerido para o GIF de animação
│
├── kinematics_planner.py         # Script 1: Geração de trajetória
├── controller_simulation.py      # Script 2: Simulação de controle dinâmico
├── gerador_simbolico_puro.py     # Script 3: Derivação simbólica da dinâmica
├── requirements.txt              # Dependências do projeto Python
└── README.md                     # Esta documentação
```

---

## 셋업 Configuração do Ambiente

### Pré-requisitos
* Python 3.8 ou superior
* Git

### Instalação
Siga os passos abaixo para configurar seu ambiente de desenvolvimento.

1.  **Clone o repositório:**
    ```bash
    git clone [https://github.com/MHC-CodeSmith/rrrp-simulation.git](https://github.com/MHC-CodeSmith/rrrp-simulation.git)
    cd rrrp-simulation
    ```

2.  **Crie e ative um ambiente virtual** (altamente recomendado):
    ```bash
    # Criar o ambiente
    python3 -m venv venv

    # Ativar no Linux/macOS
    source venv/bin/activate

    # Ativar no Windows (PowerShell/CMD)
    .\venv\Scripts\activate
    ```

3.  **Instale as dependências** listadas no `requirements.txt`:
    ```bash
    pip install -r requirements.txt
    ```
    O arquivo `requirements.txt` contém:
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

## 🚀 Fluxo de Trabalho em 3 Passos

O fluxo de trabalho foi projetado para ser sequencial e intuitivo:

**Passo 1: Planejar** ➡️ **Passo 2: Simular** ➡️ **Passo 3: Analisar (Opcional)**

1.  Execute `kinematics_planner.py` para gerar o arquivo `data/trajetoria_desejada.npy`.
2.  Execute `controller_simulation.py`, que carregará automaticamente o arquivo da etapa anterior para rodar a simulação de controle.
3.  Execute `gerador_simbolico_puro.py` a qualquer momento para obter as equações matemáticas que governam o sistema, para fins de estudo ou validação.

---

##  детальное Descrição Detalhada dos Scripts

### 1. `kinematics_planner.py`

- **Finalidade**: Criar um caminho suave para o robô seguir.
- **Entrada Principal**: Nenhum arquivo, os parâmetros da trajetória são definidos no próprio código.
- **Lógica**:
    1.  Define uma trajetória linear para a ponta do robô (TCP) no espaço.
    2.  Usa cinemática inversa simplificada para calcular os ângulos das juntas `θ₂`, `θ₃` e a extensão `d₄` necessárias para realizar esse movimento.
    3.  A junta da base `θ₁` é mantida em zero.
- **Saída Principal**: O arquivo `data/trajetoria_desejada.npy`, que é uma matriz `(N, 4)` representando a sequência de estados desejados `[θ₁, θ₂, θ₃, d₄]`.

### 2. `controller_simulation.py`

- **Finalidade**: Testar quão bem o robô consegue seguir a trajetória planejada, considerando a física do mundo real (massas, inércia, etc.).
- **Entrada Principal**: `data/trajetoria_desejada.npy`.
- **Lógica**:
    1.  **Carrega** a trajetória desejada.
    2.  **Define Ganhos PID**: As matrizes `Kp`, `Kd`, `Ki` são o "cérebro" do controlador, ajustadas para otimizar a resposta.
    3.  **Inicia o Loop de Simulação**: Para cada passo de tempo `dt`:
        - Calcula o erro entre a posição desejada e a real.
        - A lei de controle PID calcula o torque/força necessários para corrigir esse erro.
        - A dinâmica do robô (usando as matrizes `M`, `C`, `G`) determina como o robô realmente acelera sob a ação desse torque.
        - Aceleração é integrada para obter a nova velocidade e posição.
- **Saídas Principais**:
    - **Animação 3D** da trajetória (requer Robotics Toolbox).
    - **Painéis de Gráficos** detalhados para análise de performance, erro e torque.
    - **Métricas Estatísticas** impressas no terminal.

### 3. `gerador_simbolico_puro.py`

- **Finalidade**: Prover uma "prova matemática" do modelo dinâmico do robô.
- **Entrada Principal**: Nenhuma.
- **Lógica**:
    1.  Usa a biblioteca `SymPy` para tratar todas as variáveis (ângulos, massas, comprimentos) como símbolos matemáticos, não como números.
    2.  Aplica os princípios da mecânica Lagrangiana ou Newton-Euler passo a passo.
    3.  Calcula as energias cinética e potencial para derivar as equações.
- **Saída Principal**: **Expressões matemáticas** puras para as matrizes `M(q)`, `C(q, q̇)` e `G(q)`, impressas no terminal. Ideal para conferir com a literatura ou usar em um artigo científico.

---

## ▶️ Como Executar

> Certifique-se de que seu ambiente virtual (`venv`) está ativado.

```bash
# 1. Gerar a trajetória
python kinematics_planner.py

# 2. Rodar a simulação de controle
python controller_simulation.py

# 3. (Opcional) Gerar as equações simbólicas
python gerador_simbolico_puro.py
```

---

## 📊 Interpretando os Resultados

* **Arquivo `.npy`**: Carregue-o com NumPy para análise:
    ```python
    import numpy as np
    q_des = np.load('data/trajetoria_desejada.npy')
    print("Formato do array:", q_des.shape)
    ```
* **Gráficos da Simulação**: Avalie visualmente a sobreposição das curvas "Desejada" vs. "Real". Um bom controle minimiza a diferença entre elas.
* **Métricas de Performance**: Busque por `Sobressinal` baixo, `Erro de Regime` próximo de zero e `Tempo de Estabilização` rápido.
* **Painel de Análise de Erro**: O histograma do erro deve ser um pico fino e alto centrado em zero. O desvio padrão móvel deve ser baixo e estável após o transitório inicial.

---

## 📝 Licença

Este projeto é distribuído sob a Licença MIT. Veja o arquivo `LICENSE` para mais detalhes. Sinta-se à vontade para usar, modificar e distribuir este código para fins educacionais e de pesquisa.

---

## 📞 Contato

**Matheus Hipolito Carvalho**

* **E-mail**: [matheus.hipolito@usp.br](mailto:matheus.hipolito@usp.br)
* **GitHub**: [MHC-CodeSmith](https://github.com/MHC-CodeSmith)
