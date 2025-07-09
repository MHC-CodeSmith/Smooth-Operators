#!/usr/bin/env python3
"""
Teste do Backend Qt5Agg para Robotics Toolbox
"""

# Configurar backend do matplotlib para Qt5
import matplotlib
matplotlib.use("Qt5Agg")

import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH

def test_qt_backend():
    """Testa se o backend Qt5Agg está funcionando"""
    print("=== Teste do Backend Qt5Agg ===")
    
    # Verificar backend atual
    print(f"Backend atual: {matplotlib.get_backend()}")
    
    # Criar robô simples para teste
    links = [
        RevoluteDH(d=0, alpha=np.pi/2),
        RevoluteDH(a=0.3, alpha=0),
        RevoluteDH(a=0.05, alpha=np.pi/2),
        PrismaticDH(alpha=0)
    ]
    
    robot = DHRobot(links, name='RRRP')
    print(f"Robô criado: {robot.name}")
    
    # Configuração de teste
    q = [0, np.pi/4, np.pi/4, 0.1]
    print(f"Configuração de teste: {q}")
    
    # Testar plot simples
    try:
        print("Testando plot simples...")
        robot.plot(q, block=True)
        print("✓ Plot simples funcionou!")
    except Exception as e:
        print(f"✗ Erro no plot simples: {e}")
    
    # Testar animação
    try:
        print("Testando animação...")
        Q = np.array([q, q])  # Duas configurações iguais
        robot.plot(Q, block=False, dt=1.0)
        print("✓ Animação funcionou!")
    except Exception as e:
        print(f"✗ Erro na animação: {e}")

if __name__ == "__main__":
    test_qt_backend() 