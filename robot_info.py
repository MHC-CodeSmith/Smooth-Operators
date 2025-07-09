#!/usr/bin/env python3
"""
Análise Detalhada do Robô RRRP
Mostra informações sobre cinemática, jacobiano e singularidades
"""

import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
from spatialmath import SE3

def create_rrrp_robot():
    """Cria o robô RRRP com parâmetros DH"""
    d1 = 0.0    # Offset vertical da base
    l2 = 0.3    # Comprimento do link 2
    l3 = 0.05   # Comprimento do link 3
    
    links = [
        RevoluteDH(d=d1,       alpha=np.pi/2),   # Junta 1: revoluta, base vertical
        RevoluteDH(a=l2,       alpha=0),         # Junta 2: revoluta, plano vertical
        RevoluteDH(a=l3,       alpha=np.pi/2),   # Junta 3: revoluta, continua vertical
        PrismaticDH(alpha=0)                     # Junta 4: prismática, compensação
    ]
    
    # Limite do curso da junta prismática (50 cm)
    links[3].qlim = [0, 0.5]
    
    robot = DHRobot(links, name='RRRP')
    return robot, l2, l3

def analyze_robot(robot):
    """Análise completa do robô"""
    print("=== ANÁLISE DO ROBÔ RRRP ===")
    print()
    
    # Informações básicas
    print(f"Nome: {robot.name}")
    print(f"Número de juntas: {robot.n}")
    print(f"Graus de liberdade: {robot.n}")
    print()
    
    # Parâmetros DH
    print("Parâmetros DH:")
    for i, link in enumerate(robot.links):
        print(f"  Link {i+1}: theta={link.theta:.3f}, d={link.d:.3f}, a={link.a:.3f}, alpha={link.alpha:.3f}")
        if hasattr(link, 'qlim') and link.qlim is not None:
            print(f"    Limites: {link.qlim}")
    print()
    
    # Configuração zero
    q0 = np.zeros(robot.n)
    print(f"Configuração zero: {q0}")
    
    # Cinemática direta na configuração zero
    T0 = robot.fkine(q0)
    print(f"Pose na configuração zero:")
    print(f"  Posição: {T0.t}")
    print(f"  Orientação: {T0.R}")
    print()
    
    # Jacobiano na configuração zero
    J0 = robot.jacob0(q0)
    print(f"Jacobiano na configuração zero:")
    print(J0)
    print()
    
    # Determinante do jacobiano (para detectar singularidades)
    det_J = np.linalg.det(J0[:3, :3])  # Apenas parte linear
    print(f"Determinante do jacobiano (parte linear): {det_J:.6f}")
    
    if abs(det_J) < 1e-6:
        print("⚠️  SINGULARIDADE DETECTADA na configuração zero!")
    else:
        print("✓ Configuração zero não é singular")
    print()
    
    # Análise de workspace
    print("=== ANÁLISE DE WORKSPACE ===")
    
    # Testa algumas configurações
    test_configs = [
        [0, 0, 0, 0],           # Configuração zero
        [0, np.pi/2, 0, 0.2],   # Junta 2 em 90°
        [0, np.pi/4, np.pi/4, 0.1],  # Configuração intermediária
        [0, 0, np.pi/2, 0.3],   # Junta 3 em 90°
    ]
    
    for i, q in enumerate(test_configs):
        T = robot.fkine(q)
        J = robot.jacob0(q)
        det_J = np.linalg.det(J[:3, :3])
        
        print(f"Configuração {i+1}: q = {q}")
        print(f"  Posição: {T.t}")
        print(f"  Det(J): {det_J:.6f}")
        if abs(det_J) < 1e-6:
            print("  ⚠️  SINGULARIDADE!")
        print()
    
    # Análise de singularidades
    print("=== ANÁLISE DE SINGULARIDADES ===")
    print("O robô RRRP pode ter singularidades quando:")
    print("1. Junta 2 está em 0° ou 180° (singularidade de braço)")
    print("2. Junta 3 está em 0° ou 180° (singularidade de pulso)")
    print("3. Junta prismática atinge seus limites")
    print()
    
    # Verifica singularidades em diferentes ângulos da junta 2
    print("Verificando singularidades da junta 2:")
    for angle in [0, np.pi/6, np.pi/4, np.pi/3, np.pi/2, 2*np.pi/3, 3*np.pi/4, 5*np.pi/6, np.pi]:
        q = [0, angle, 0, 0.1]
        J = robot.jacob0(q)
        det_J = np.linalg.det(J[:3, :3])
        print(f"  q2 = {np.degrees(angle):3.0f}°: Det(J) = {det_J:.6f}")
        if abs(det_J) < 1e-6:
            print("    ⚠️  SINGULARIDADE!")
    print()
    
    return robot

def test_trajectory(robot):
    """Testa a trajetória de soldagem"""
    print("=== TESTE DA TRAJETÓRIA DE SOLDAGEM ===")
    
    # Gera trajetória similar ao script principal
    N = 20  # Menos pontos para teste
    x_desejado = np.linspace(0.2, 0.5, N)
    
    q1 = 0.0
    q2_vals = np.linspace(np.pi/2, 0.0, N)
    q3_vals = np.pi/2 - q2_vals
    
    Q = np.zeros((N, 4))
    
    for i in range(N):
        q2 = q2_vals[i]
        q3 = q3_vals[i]
        q4 = x_desejado[i] - 0.3 * np.cos(q2)  # l2 = 0.3
        Q[i, :] = [q1, q2, q3, q4]
    
    print(f"Trajetória gerada com {N} pontos")
    print("Primeiras configurações:")
    for i in range(min(5, N)):
        print(f"  t={i}: q = {Q[i]}")
    
    # Verifica singularidades ao longo da trajetória
    print("\nVerificando singularidades na trajetória:")
    singular_points = []
    
    for i in range(N):
        J = robot.jacob0(Q[i])
        det_J = np.linalg.det(J[:3, :3])
        if abs(det_J) < 1e-6:
            singular_points.append(i)
            print(f"  ⚠️  Singularidade no ponto {i}: q = {Q[i]}")
    
    if not singular_points:
        print("  ✓ Trajetória livre de singularidades!")
    else:
        print(f"  ⚠️  {len(singular_points)} pontos singulares encontrados")
    
    print()
    return Q

def main():
    """Função principal"""
    try:
        # Cria e analisa o robô
        robot, l2, l3 = create_rrrp_robot()
        analyze_robot(robot)
        
        # Testa a trajetória
        Q = test_trajectory(robot)
        
        print("=== RESUMO ===")
        print("✓ Robô RRRP criado com sucesso")
        print("✓ Análise cinemática concluída")
        print("✓ Trajetória de soldagem testada")
        print()
        print("O robô está pronto para simulação!")
        
    except Exception as e:
        print(f"Erro na análise: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 