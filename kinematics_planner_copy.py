#!/usr/bin/env python3
"""
Simulação do Robô RRRP - kinematics_planner_diagonal.py (Versão Modificada)

Autor: André MK022 (Cuca) - Adaptado por Gemini para gerar trajetória diagonal.
Descrição: Implementa a geração de uma trajetória em linha reta no espaço cartesiano.
           O robô começa na posição "em pé" com o atuador prismático estendido e
           move seu efetuador em uma linha reta até uma posição "abaixada" com o
           atuador prismático retraído.

Requerimentos: numpy, roboticstoolbox-python
Instalação: pip install numpy roboticstoolbox-python
"""
import numpy as np

# Adicione esta importação no início do seu arquivo
import roboticstoolbox as rtb
# A cinemática inversa é complexa, então o Robotics Toolbox é essencial aqui.
try:
    from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH, SerialLink
    from spatialmath import SE3 # Para criar a matriz de pose de destino
    ROBOTICS_TOOLBOX_AVAILABLE = True
except ImportError:
    print("AVISO: Robotics Toolbox não encontrado (pip install roboticstoolbox-python).")
    print("Este script modificado PRECISA do toolbox para calcular a cinemática inversa.")
    ROBOTICS_TOOLBOX_AVAILABLE = False

def create_rrrp_robot_from_dh():
    """
    Cria um robô RRRP com base nos parâmetros DH do script MATLAB.
    Esta função permanece a mesma.
    """
    l2 = 0.3
    l3 = 0.03
    d4_lim = [-0.02, 0.105]

    # Os links DH que definem a cinemática do robô
    links = [
        RevoluteDH(d=0, a=0, alpha=np.pi/2),
        RevoluteDH(d=0, a=l2, alpha=0),
        RevoluteDH(d=0, a=l3, alpha=np.pi/2),
        PrismaticDH(theta=0, a=0, alpha=0, qlim=d4_lim)
    ]
    robot_model = SerialLink(links, name='RRRP_Orca_DH_Diagonal')
    
    return robot_model, l2, l3, d4_lim


def generate_diagonal_line_trajectory(robot, d4_lim, N=100):
    """
    Gera uma trajetória em linha reta usando Cinemática Inversa.
    """
    if not ROBOTICS_TOOLBOX_AVAILABLE:
        print("✗ ERRO: A geração de trajetória diagonal requer o Robotics Toolbox.")
        return np.array([])

    print("Gerando trajetória diagonal com cinemática inversa...")

    # --- DEFINIÇÃO DO MOVIMENTO DESEJADO ---
    # O robô deve começar "em pé" e ir "abaixando" enquanto retrai a garra.

    # Posição INICIAL: "Em pé" (q2=pi/2) com a prismática estendida (q4=max).
    q_inicial = np.array([0, np.pi/2, 0, d4_lim[1]])

    # Posição FINAL: "Deitado" (q2=0) com a prismática retraída (q4=min).
    q_final = np.array([0, 0, np.pi/2, d4_lim[0]])

    # 2. Calcular as poses cartesianas (X,Y,Z) para o início e o fim
    T_inicial = robot.fkine(q_inicial)
    T_final = robot.fkine(q_final)

    print(f"Posição Cartesiana Inicial: {np.round(T_inicial.t, 3)}")
    print(f"Posição Cartesiana Final:   {np.round(T_final.t, 3)}")

    # 3. Gerar a trajetória em LINHA RETA no espaço cartesiano
    traj_cartesiana = rtb.ctraj(T_inicial, T_final, N)

    # 4. Calcular a Cinemática Inversa para cada ponto da trajetória
    solucao_ik = robot.ikine_LM(traj_cartesiana, q0=q_inicial)

    # 5. Checar se a solução da IK foi bem-sucedida
    if not solucao_ik.success:
         print("✗ AVISO: A solução da cinemática inversa pode ter falhado para a trajetória.")

    return solucao_ik.q

def main():
    """Função principal para gerar e salvar a trajetória."""
    print("=== Planejador de Trajetória Diagonal (Cinemática Inversa) ===")

    if not ROBOTICS_TOOLBOX_AVAILABLE:
        return

    robot, _, _, d4_lim = create_rrrp_robot_from_dh()
    
    q_desejado = generate_diagonal_line_trajectory(robot, d4_lim, N=100)

    if q_desejado.size == 0:
        print(f"✗ ERRO: Não foi possível gerar a trajetória.")
        return

    np.save('trajetoria_desejada.npy', q_desejado)
    print(f"\n✓ Trajetória com {len(q_desejado)} pontos foi salva em 'trajetoria_desejada.npy'.")
    print("Esta trajetória fará o efetuador do robô se mover em uma linha reta no espaço.")

if __name__ == "__main__":
    main()