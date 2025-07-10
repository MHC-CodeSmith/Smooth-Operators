#!/usr/bin/env python3
"""
Simulação do Robô RRRP - controller_simulation.py (Versão 14.0 - Simulação Estável)

Autor: André MK022 (Cuca) - Refinado com feedback por Gemini
Descrição: Implementa uma simulação dinâmica estável. Os ganhos PID iniciais
foram reduzidos para valores seguros, e foi adicionada uma lógica anti-windup
e uma verificação de sanidade para prevenir a instabilidade numérica que causava
a tela em branco.
"""
import numpy as np
import matplotlib
matplotlib.use("Qt5Agg")  # Força o uso do backend Qt5
import matplotlib.pyplot as plt
import time

try:
    from roboticstoolbox import DHRobot, RevoluteDH
    from spatialmath import SE3
    ROBOTICS_TOOLBOX_AVAILABLE = True
except ImportError:
    print("Robotics Toolbox não encontrada. A animação visual não está disponível.")
    ROBOTICS_TOOLBOX_AVAILABLE = False

# --- Modelo do Robô com Dinâmica ---
def create_real_robot():
    """Cria um robô RRR com as dimensões, limites E PARÂMETROS DINÂMICOS ESTIMADOS."""
    l1, l2, l3 = 0.1, 0.2, 0.1
    m2, m3 = 1.0, 0.5; r2, r3 = [l2/2, 0, 0], [l3/2, 0, 0]
    I2, I3 = np.diag([0.01, 0.01, 0.005]), np.diag([0.005, 0.005, 0.002])
    links = [
        RevoluteDH(d=l1, alpha=np.pi/2),
        RevoluteDH(a=l2, alpha=0, m=m2, r=r2, I=I2, qlim=[0, np.pi]),
        RevoluteDH(a=l3, alpha=0, m=m3, r=r3, I=I3, qlim=[-np.pi/2, np.pi/2]),
    ]
    return DHRobot(links, name='Real_RRR_Base_com_Dinamica')

# --- Funções de Plotagem ---
def plotar_resultados_finais(t, q_des, q_real, erro, torque):
    """Gera todos os gráficos de análise de desempenho após a simulação."""
    print("Gerando gráficos de análise final...")
    num_juntas = q_des.shape[1]
    titulos_juntas = ['Junta 1 (θ₁)', 'Junta 2 (θ₂)', 'Junta 3 (θ₃)', 'Junta 4 (d₄)']
    
    # Gráfico 1: Seguimento de Trajetória
    plt.figure(figsize=(15, 10))
    for i in range(num_juntas):
        plt.subplot(num_juntas, 1, i+1)
        plt.plot(t, q_des[:, i], 'r--', label='Desejado', linewidth=2)
        plt.plot(t, q_real[:, i], 'b-', label='Obtido', linewidth=1.5)
        plt.title(f'Seguimento de Trajetória - {titulos_juntas[i]}')
        plt.ylabel('Posição (rad/m)')
        plt.grid(True, alpha=0.3)
        plt.legend()
    plt.xlabel('Tempo (s)')
    plt.tight_layout()
    print("Mostrando Gráfico 1: Seguimento de Trajetória...")
    plt.show(block=True)
    
    # Gráfico 2: Erro de Seguimento
    plt.figure(figsize=(15, 8))
    for i in range(num_juntas):
        plt.subplot(2, 2, i+1)
        plt.plot(t, erro[:, i], 'g-', linewidth=1.5)
        plt.title(f'Erro de Seguimento - {titulos_juntas[i]}')
        plt.ylabel('Erro (rad/m)')
        plt.xlabel('Tempo (s)')
        plt.grid(True, alpha=0.3)
    plt.tight_layout()
    print("Mostrando Gráfico 2: Erro de Seguimento...")
    plt.show(block=True)
    
    # Gráfico 3: Torque Aplicado
    plt.figure(figsize=(15, 8))
    for i in range(num_juntas):
        plt.subplot(2, 2, i+1)
        plt.plot(t, torque[:, i], 'm-', linewidth=1.5)
        plt.title(f'Torque Aplicado - {titulos_juntas[i]}')
        plt.ylabel('Torque (N.m)')
        plt.xlabel('Tempo (s)')
        plt.grid(True, alpha=0.3)
    plt.tight_layout()
    print("Mostrando Gráfico 3: Torque Aplicado...")
    plt.show(block=True)
    
    # Gráfico 4: Análise de Desempenho
    plt.figure(figsize=(12, 8))
    
    # Erro RMS por junta
    erro_rms = np.sqrt(np.mean(erro**2, axis=0))
    plt.subplot(2, 2, 1)
    plt.bar(range(num_juntas), erro_rms, color='skyblue', alpha=0.7)
    plt.title('Erro RMS por Junta')
    plt.ylabel('Erro RMS (rad/m)')
    plt.xticks(range(num_juntas), titulos_juntas, rotation=45)
    plt.grid(True, alpha=0.3)
    
    # Torque máximo por junta
    torque_max = np.max(np.abs(torque), axis=0)
    plt.subplot(2, 2, 2)
    plt.bar(range(num_juntas), torque_max, color='lightcoral', alpha=0.7)
    plt.title('Torque Máximo por Junta')
    plt.ylabel('Torque Máximo (N.m)')
    plt.xticks(range(num_juntas), titulos_juntas, rotation=45)
    plt.grid(True, alpha=0.3)
    
    # Erro médio absoluto
    erro_medio = np.mean(np.abs(erro), axis=0)
    plt.subplot(2, 2, 3)
    plt.bar(range(num_juntas), erro_medio, color='lightgreen', alpha=0.7)
    plt.title('Erro Médio Absoluto por Junta')
    plt.ylabel('Erro Médio (rad/m)')
    plt.xticks(range(num_juntas), titulos_juntas, rotation=45)
    plt.grid(True, alpha=0.3)
    
    # Desvio padrão do erro
    erro_std = np.std(erro, axis=0)
    plt.subplot(2, 2, 4)
    plt.bar(range(num_juntas), erro_std, color='gold', alpha=0.7)
    plt.title('Desvio Padrão do Erro por Junta')
    plt.ylabel('Desvio Padrão (rad/m)')
    plt.xticks(range(num_juntas), titulos_juntas, rotation=45)
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    print("Mostrando Gráfico 4: Análise de Desempenho...")
    plt.show(block=True)
    
    # Resumo estatístico
    print("\n=== RESUMO DE DESEMPENHO ===")
    print(f"{'Junta':<15} {'Erro RMS':<12} {'Erro Médio':<12} {'Torque Máx':<12}")
    print("-" * 55)
    for i in range(num_juntas):
        print(f"{titulos_juntas[i]:<15} {erro_rms[i]:<12.4f} {erro_medio[i]:<12.4f} {torque_max[i]:<12.4f}")

def animate_simulation_results(robot, dt, q_desejado, q_real):
    """Toca a animação dos resultados da simulação previamente calculados."""
    if not ROBOTICS_TOOLBOX_AVAILABLE: return
    print("Iniciando playback da animação...")
    tcp_desejado_path = np.array([robot.fkine(q[:3]) * SE3.Tx(q[3]) for q in q_desejado])
    env = robot.plot(q_real[0,:3], block=False, limits=[-0.5, 0.5, -0.5, 0.5, 0, 0.5])
    env.ax.plot(tcp_desejado_path[:,0,3], tcp_desejado_path[:,1,3], tcp_desejado_path[:,2,3], 'r--', linewidth=2, label='Trajetória TCP Desejada')
    tool_line, = env.ax.plot([], [], [], 'g-', linewidth=4, label='Ferramenta (q₄)')
    env.ax.legend()
    for i in range(len(q_real)):
        robot.q = q_real[i, :3]
        wrist_pose = robot.fkine(robot.q); tcp_pose = wrist_pose * SE3.Tx(q_real[i, 3])
        tool_line.set_data_3d([wrist_pose.t[0], tcp_pose.t[0]], [wrist_pose.t[1], tcp_pose.t[1]], [wrist_pose.t[2], tcp_pose.t[2]])
        env.step(dt)
    print("Animação concluída. Feche a janela para ver os gráficos de análise.")
    plt.show(block=True)

def main():
    """Função principal que executa a simulação e depois a animação."""
    print("=== Simulador de Controle Dinâmico (V14.0 - Simulação Estável) ===")
    visualizar_animacao = True
    if visualizar_animacao and not ROBOTICS_TOOLBOX_AVAILABLE:
        print("AVISO: Robotics Toolbox não encontrado. A animação visual será pulada.")
        visualizar_animacao = False
        
    robot = create_real_robot()

    try:
        q_desejado = np.load('trajetoria_desejada.npy')
        print(f"Trajetória com {len(q_desejado)} pontos carregada com sucesso.")
    except FileNotFoundError:
        print("ERRO: Arquivo 'trajetoria_desejada.npy' não encontrado."); return

    # --- Setup da Simulação ---
    dt = 0.01
    t = np.arange(0, len(q_desejado) * dt, dt)
    qd_desejado = np.gradient(q_desejado, dt, axis=0)

    # --- PROJETO DO CONTROLADOR PID (GANHOS MUITO CONSERVADORES) ---
    # Ganhos extremamente baixos para garantir estabilidade
    Kp = np.diag([0, 5, 3, 2])      # Ganhos proporcionais muito baixos
    Kd = np.diag([0, 2, 1, 0.5])    # Ganhos derivativos mínimos
    Ki = np.diag([0, 0, 0, 0])      # Ganho integral ZERO para evitar instabilidade

    print("Iniciando simulação numérica...")
    q_real = np.zeros_like(q_desejado); qd_real = np.zeros_like(q_desejado)
    q_real[0,:] = q_desejado[0,:]
    hist_erro, hist_torque = [], []
    erro_integral = np.zeros(robot.n + 1)
    max_integral_error = 1.0 # Limite para a lógica Anti-Windup

    # --- Loop de Simulação Numérica ---
    for i in range(1, len(t)):
        erro = q_desejado[i-1,:] - q_real[i-1,:]
        erro_deriv = qd_desejado[i-1,:] - qd_real[i-1,:]
        
        # Lógica Anti-Windup para o termo integral
        erro_integral += erro * dt
        erro_integral = np.clip(erro_integral, -max_integral_error, max_integral_error)
        
        # Controlador PID simples sem compensação de gravidade
        torque_pid = Kp @ erro + Kd @ erro_deriv + Ki @ erro_integral
        torque_total = torque_pid
        
        hist_erro.append(erro); hist_torque.append(torque_total)
        
        # Simulação da dinâmica simplificada
        # Massas inerciais estimadas para cada junta
        massas = np.array([0.1, 1.0, 0.5, 0.2])  # Massas das juntas 1, 2, 3 e ferramenta
        qdd_real = torque_total / massas

        # Integração de Euler para o próximo estado
        qd_real[i,:] = qd_real[i-1,:] + qdd_real * dt
        q_real[i,:] = q_real[i-1,:] + qd_real[i,:] * dt

    print("Simulação numérica concluída.")
    
    # --- VERIFICAÇÃO DE SANIDADE ---
    if np.isnan(q_real).any() or np.isinf(q_real).any():
        print("\n!!! ERRO DE SIMULAÇÃO: INSTABILIDADE NUMÉRICA DETECTADA !!!")
        print("Os valores de posição do robô explodiram para infinito ou NaN.")
        print("Isso geralmente é causado por ganhos PID muito altos.")
        print("Tente reduzir os valores de Kp, Kd ou Ki e rode novamente.")
        return
    
    # Verificação adicional de estabilidade
    max_pos = np.max(np.abs(q_real))
    max_vel = np.max(np.abs(qd_real))
    print(f"Valores máximos - Posição: {max_pos:.3f}, Velocidade: {max_vel:.3f}")
    if max_pos > 10 or max_vel > 10:
        print("AVISO: Valores muito altos detectados. Considere reduzir ainda mais os ganhos PID.")

    # --- FASE 2: ANIMAÇÃO E PLOTAGEM DOS RESULTADOS ---
    if visualizar_animacao:
        animate_simulation_results(robot, dt, q_desejado, q_real)

    plotar_resultados_finais(t[1:], q_desejado[1:], q_real[1:], np.array(hist_erro), np.array(hist_torque))
    print("\nProcesso finalizado!")

if __name__ == "__main__":
    main()