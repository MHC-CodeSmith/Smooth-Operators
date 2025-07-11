#!/usr/bin/env python3
"""
Simulação do Robô RRRP - controller_simulation.py (Versão 2.1 - Correção de Bug)

Autor: André MK022 (Cuca) - Adaptado por Gemini com base em novo modelo
Descrição: Corrige um bug na função de animação que usava 'fkine_all'
incorretamente. Substituído por 'fkine' para processar a trajetória.
"""
import numpy as np
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
import time

try:
    from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH, SerialLink
    from spatialmath import SE3
    ROBOTICS_TOOLBOX_AVAILABLE = True
except ImportError:
    print("Robotics Toolbox não encontrada. A animação visual não está disponível.")
    ROBOTICS_TOOLBOX_AVAILABLE = False

# --- Modelo do Robô com Dinâmica ---
def create_rrrp_robot_from_dh():
    """
    Cria um robô RRRP com base nos parâmetros DH e adiciona massas reais.
    """
    l2, l3 = 0.3, 0.05
    
    m1, m2, m3, m4 = 0.08159, 0.201456, 0.03425, 0.0252
    
    r2, r3 = [l2/2, 0, 0], [l3/2, 0, 0]
    I1 = np.diag([0.001, 0.001, 0.001])
    I2 = np.diag([0.01, 0.01, 0.005])
    I3 = np.diag([0.005, 0.005, 0.002])
    
    links = [
        RevoluteDH(d=0, a=0, alpha=np.pi/2, m=m1, I=I1),
        RevoluteDH(d=0, a=l2, alpha=0, m=m2, r=r2, I=I2),
        RevoluteDH(d=0, a=l3, alpha=np.pi/2, m=m3, r=r3, I=I3),
        PrismaticDH(theta=0, a=0, alpha=0, qlim=[0, 0.5], m=m4)
    ]
    
    return SerialLink(links, name='RRRP_Orca_DH_Dinamica')

# --- Funções de Plotagem (sem alterações) ---
def plotar_resultados_finais(t, q_des, q_real, erro, torque):
    print("Gerando gráficos de análise final...")
    num_juntas = q_des.shape[1]
    titulos_juntas = ['Junta 1 (θ₁)', 'Junta 2 (θ₂)', 'Junta 3 (θ₃)', 'Junta 4 (d₄)']
    plt.figure(figsize=(15, 10))
    for i in range(num_juntas):
        plt.subplot(num_juntas, 1, i+1); plt.plot(t, q_des[:, i], 'r--', label='Desejado', linewidth=2); plt.plot(t, q_real[:, i], 'b-', label='Obtido', linewidth=1.5); plt.title(f'Seguimento de Trajetória - {titulos_juntas[i]}'); plt.ylabel('Posição (rad/m)'); plt.grid(True, alpha=0.3); plt.legend()
    plt.xlabel('Tempo (s)'); plt.tight_layout(); plt.show(block=True)
    plt.figure(figsize=(15, 8))
    for i in range(num_juntas):
        plt.subplot(2, 2, i+1); plt.plot(t, erro[:, i], 'g-', linewidth=1.5); plt.title(f'Erro de Seguimento - {titulos_juntas[i]}'); plt.ylabel('Erro (rad/m)'); plt.xlabel('Tempo (s)'); plt.grid(True, alpha=0.3)
    plt.tight_layout(); plt.show(block=True)
    plt.figure(figsize=(15, 8))
    for i in range(num_juntas):
        plt.subplot(2, 2, i+1); plt.plot(t, torque[:, i], 'm-', linewidth=1.5); plt.title(f'Torque Aplicado - {titulos_juntas[i]}'); plt.ylabel('Torque/Força (N.m/N)'); plt.xlabel('Tempo (s)'); plt.grid(True, alpha=0.3)
    plt.tight_layout(); plt.show(block=True)

def animate_simulation_results(robot, dt, q_desejado, q_real):
    """Anima os resultados da simulação para o robô RRRP."""
    if not ROBOTICS_TOOLBOX_AVAILABLE: return
    print("Iniciando playback da animação...")
    
    env = robot.plot(q_real[0, :], block=False, limits=[-0.5, 0.5, -0.5, 0.5, -0.1, 0.6])
    
    # --- CORRIGIDO AQUI ---
    # Usar robot.fkine() que aceita uma matriz (N, n) de pontos de trajetória.
    # robot.fkine_all() era incorreto e causou o erro.
    T_des_path = robot.fkine(q_desejado)
    
    # O resto do código funciona como esperado com a correção acima
    tcp_des_path = T_des_path.t
    env.ax.plot(tcp_des_path[:,0], tcp_des_path[:,1], tcp_des_path[:,2], 'r--', linewidth=2, label='Trajetória TCP Desejada')
    env.ax.legend()

    for i in range(len(q_real)):
        robot.q = q_real[i, :]
        env.step(dt)
        
    print("Animação concluída. Feche a janela para ver os gráficos de análise.")
    plt.show(block=True)

def main():
    """Função principal que executa a simulação e depois a animação."""
    print("=== Simulador de Controle Dinâmico (V2.1 - Correção de Bug) ===")
    visualizar_animacao = True
    if visualizar_animacao and not ROBOTICS_TOOLBOX_AVAILABLE:
        print("AVISO: Robotics Toolbox não encontrado. A animação visual será pulada.")
        visualizar_animacao = False
        
    robot = create_rrrp_robot_from_dh()

    try:
        q_desejado = np.load('trajetoria_desejada.npy')
        print(f"Trajetória com {len(q_desejado)} pontos carregada com sucesso.")
    except FileNotFoundError:
        print("ERRO: Arquivo 'trajetoria_desejada.npy' não encontrado."); return

    dt = 0.01; t = np.arange(0, len(q_desejado) * dt, dt)
    qd_desejado = np.gradient(q_desejado, dt, axis=0)

    # --- !! ATENÇÃO: GANHOS PID PRECISAM DE SINTONIA !! ---
    Kp = np.diag([10, 20, 15, 5]) 
    Kd = np.diag([2,  5,  3, 1])
    Ki = np.diag([0,  0,  0, 0])

    print("Iniciando simulação numérica...")
    q_real = np.zeros_like(q_desejado); qd_real = np.zeros_like(q_desejado)
    q_real[0,:] = q_desejado[0,:]; hist_erro, hist_torque = [], []
    erro_integral = np.zeros(robot.n); max_integral_error = 1.0

    massas_reais = np.array([0.08159, 0.201456, 0.03425, 0.0252])

    for i in range(1, len(t)):
        erro = q_desejado[i-1,:] - q_real[i-1,:]; erro_deriv = qd_desejado[i-1,:] - qd_real[i-1,:]
        erro_integral += erro * dt; erro_integral = np.clip(erro_integral, -max_integral_error, max_integral_error)
        
        torque_pid = Kp @ erro + Kd @ erro_deriv + Ki @ erro_integral
        torque_total = torque_pid
        
        hist_erro.append(erro); hist_torque.append(torque_total)
        
        qdd_real = torque_total / massas_reais
        qd_real[i,:] = qd_real[i-1,:] + qdd_real * dt
        q_real[i,:] = q_real[i-1,:] + qd_real[i,:] * dt

    print("Simulação numérica concluída.")
    
    if np.isnan(q_real).any() or np.isinf(q_real).any():
        print("\n!!! ERRO: INSTABILIDADE NUMÉRICA DETECTADA !!!"); return

    if visualizar_animacao:
        animate_simulation_results(robot, dt, q_desejado, q_real)

    plotar_resultados_finais(t[1:], q_desejado[1:], q_real[1:], np.array(hist_erro), np.array(hist_torque))
    print("\nProcesso finalizado!")

if __name__ == "__main__":
    main()