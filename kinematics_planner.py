#!/usr/bin/env python3
"""
Simulação do Robô RRRP - kinematics_planner.py (Versão 3.0 - Trajetória Complexa)

Autor: André MK022 (Cuca) - Adaptado por Gemini com base em nova regra de trajetória
Descrição: Implementa a geração de uma trajetória complexa onde a junta prismática (d4)
se move em função do ângulo da junta 2 (theta2), conforme especificado.
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
    Cria um robô RRRP com base nos parâmetros DH e dimensões reais.
    """
    # --- ALTERADO AQUI: Dimensões Reais Atualizadas ---
    # L2=300mm, L3=29.5mm
    l2 = 0.3
    l3 = 0.0295 # Valor atualizado
    
    # Limites de d4: -20mm a 105mm
    d4_lim = [-0.02, 0.105]
    
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

def generate_custom_trajectory(N=200):
    """
    Gera a trajetória onde d4 depende de theta2.
    - theta2 varre de 0 a 90 graus.
    - d4 começa em 105mm, vai para -20mm (em theta2=45), e volta para 105mm (em theta2=90).
    - q3 = pi/2 - q2 para manter a orientação.
    """
    print("Gerando trajetória customizada (d4 em função de theta2)...")

    # 1. Definir o movimento da junta 2 (theta2) de 0 a 90 graus (pi/2)
    q2_vals = np.linspace(0, np.pi/2, N)
    
    # 2. Manter a orientação do efetuador com a regra q3 = pi/2 - q2
    q3_vals = np.pi/2 - q2_vals
    
    # 3. Manter a junta 1 (base) fixa
    q1_vals = np.zeros(N)
    
    # 4. Gerar a trajetória para a junta 4 (d4)
    q4_vals = np.zeros(N)
    ponto_intermediario = N // 2 # Ponto onde theta2 é 45 graus
    
    # Parte 1: De 105mm a -20mm (enquanto theta2 vai de 0 a 45 graus)
    q4_vals[:ponto_intermediario] = np.linspace(0.105, -0.02, ponto_intermediario)
    
    # Parte 2: De -20mm de volta a 105mm (enquanto theta2 vai de 45 a 90 graus)
    q4_vals[ponto_intermediario:] = np.linspace(-0.02, 0.105, N - ponto_intermediario)

    # 5. Monta a matriz de trajetória Q
    Q = np.vstack([q1_vals, q2_vals, q3_vals, q4_vals]).T
    
    return Q

def main():
    """Função principal para gerar e salvar a trajetória."""
    print("=== Planejador de Trajetória Cinemática (V3.0) ===")
    
    q_desejado = generate_custom_trajectory()

    if q_desejado.size == 0:
        print(f"✗ ERRO: Não foi possível gerar a trajetória.")
        return

    np.save('trajetoria_desejada.npy', q_desejado)
    print(f"\n✓ Trajetória customizada com {len(q_desejado)} pontos foi salva em 'trajetoria_desejada.npy'.")

if __name__ == "__main__":
    main()