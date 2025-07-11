#!/usr/bin/env python3
"""
Simulação do Robô RRRP - kinematics_planner_matlab.py (Versão MATLAB)

Autor: André MK022 (Cuca) - Adaptado por Gemini para replicar a lógica do MATLAB.
Descrição: Implementa a geração de trajetória seguindo exatamente as regras e
parâmetros do script MATLAB fornecido pelo usuário.
"""
import numpy as np

try:
    from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH, SerialLink
    ROBOTICS_TOOLBOX_AVAILABLE = True
except ImportError:
    print("AVISO: Robotics Toolbox não encontrado. O script funcionará, mas sem a criação do objeto robô.")
    ROBOTICS_TOOLBOX_AVAILABLE = False

def create_rrrp_robot_from_dh():
    """
    Cria um robô RRRP com base nos parâmetros DH do script MATLAB.
    """
    # --- PARÂMETROS IGUAIS AO MATLAB ---
    l2 = 0.3
    l3 = 0.03  # Valor usado no MATLAB
    d4_lim = [0, 0.105] # Limites usados no MATLAB
    
    links = []
    if ROBOTICS_TOOLBOX_AVAILABLE:
        links = [
            RevoluteDH(d=0, a=0, alpha=np.pi/2),
            RevoluteDH(d=0, a=l2, alpha=0),
            RevoluteDH(d=0, a=l3, alpha=np.pi/2),
            PrismaticDH(theta=0, a=0, alpha=0, qlim=d4_lim)
        ]
        robot_model = SerialLink(links, name='RRRP_Orca_DH')
    else:
        robot_model = None

    return robot_model, l2, l3

def generate_matlab_like_trajectory(N=100):
    """
    Gera a trajetória usando a lógica de cálculo do script MATLAB.
    """
    print("Gerando trajetória com a lógica do MATLAB...")

    # Parâmetros usados no cálculo
    l2 = 0.3
    
    # 1. Trajetória cartesiana desejada para a posição X (como no MATLAB)
    x_desejado = np.linspace(0.2, 0.2 + 0.3, N)
    
    # 2. Movimentos das juntas definidos como no MATLAB
    q1_vals = np.zeros(N)
    q2_vals = np.linspace(np.pi/2, 0, N) # de 90 a 0 graus
    q3_vals = np.pi/2 - q2_vals
    
    # 3. Cálculo de q4 usando a fórmula exata do MATLAB
    # q4 = (x_desejado - l2*cos(q2))
    q4_vals = x_desejado - l2 * np.cos(q2_vals)
    
    # 4. Monta a matriz de trajetória Q
    Q = np.vstack([q1_vals, q2_vals, q3_vals, q4_vals]).T
    
    return Q

def main():
    """Função principal para gerar e salvar a trajetória."""
    print("=== Planejador de Trajetória (Lógica MATLAB) ===")
    
    q_desejado = generate_matlab_like_trajectory()

    if q_desejado.size == 0:
        print(f"✗ ERRO: Não foi possível gerar a trajetória.")
        return

    np.save('trajetoria_desejada.npy', q_desejado)
    print(f"\n✓ Trajetória com {len(q_desejado)} pontos foi salva em 'trajetoria_desejada.npy'.")

if __name__ == "__main__":
    main()