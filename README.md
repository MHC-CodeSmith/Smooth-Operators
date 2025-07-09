# Simulação do Robô RRRP para Soldagem

Este projeto implementa uma simulação em Python do robô RRRP (3 juntas rotacionais + 1 junta prismática) para aplicação de soldagem, equivalente ao código MATLAB original.

## Descrição

O robô RRRP é projetado para soldagem com as seguintes características:
- **Junta 1**: Revoluta, base vertical
- **Junta 2**: Revoluta, gira no plano vertical (30cm)
- **Junta 3**: Revoluta, continua no mesmo plano vertical (5cm)
- **Junta 4**: Prismática, compensação do consumo do eletrodo (curso até 50cm)

A trajetória implementada compensa o consumo do eletrodo mantendo o movimento em X linear no tempo, enquanto a junta 2 roda de 90° a 0° e a junta 3 mantém a orientação.

## Instalação

1. Clone o repositório:
```bash
git clone <url-do-repositorio>
cd Smooth-Operators
```

2. Instale as dependências:
```bash
pip install -r requirements.txt
```

Ou instale manualmente:
```bash
pip install numpy matplotlib roboticstoolbox-python spatialmath-python swift-sim
```

## Uso

Execute o script principal:
```bash
python rrrp_robot.py
```

### Funcionalidades

- **Com Robotics Toolbox**: Animação 3D completa do robô seguindo a trajetória
- **Sem Robotics Toolbox**: Plot 3D básico da trajetória usando matplotlib

### Parâmetros Configuráveis

- `l2 = 0.3m`: Comprimento do link 2
- `l3 = 0.05m`: Comprimento do link 3
- `N = 100`: Resolução da trajetória
- `x_desejado`: Trajetória linear de 0.2m a 0.5m

## Estrutura do Código

- `rrrp_robot.py`: Script principal com simulação completa
- `requirements.txt`: Dependências do projeto
- `README.md`: Este arquivo

## Características Técnicas

### Cinemática
- Parâmetros DH padrão
- Cinemática direta implementada
- Trajetória de compensação do eletrodo

### Controle
- Trajetória pré-definida
- Compensação automática com junta prismática
- Evita singularidades através do projeto da trajetória

### Visualização
- Animação 3D interativa (com Robotics Toolbox)
- Plot estático da trajetória (fallback)
- Gráficos de posição e orientação

## Autor

André MK022 (Cuca)

## Notas

- O robô é projetado para evitar singularidades através da trajetória específica
- A junta prismática compensa o consumo do eletrodo mantendo X linear
- As dimensões são baseadas em um eletrodo de tamanho real 