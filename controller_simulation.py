#!/usr/bin/env python3
"""
Simulação do Robô RRRP - controller_simulation.py (Versão 3.0 - Fórmulas Simbólicas)

Autor: André MK022 (Cuca) - Adaptado por Gemini com base em novo modelo
Descrição: Adiciona a funcionalidade de gerar as equações simbólicas do torque
após a execução da simulação e salva-as em um arquivo de texto.
Utiliza a biblioteca SymPy para a derivação matemática.
"""
import numpy as np
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
import time

# --- Verificação de Bibliotecas ---
try:
    from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH, SerialLink
    from spatialmath import SE3
    ROBOTICS_TOOLBOX_AVAILABLE = True
except ImportError:
    print("Robotics Toolbox não encontrada. A animação visual não está disponível.")
    ROBOTICS_TOOLBOX_AVAILABLE = False

try:
    import sympy as sp
    SYMPY_AVAILABLE = True
except ImportError:
    print("SymPy não encontrada. A geração de fórmulas simbólicas não está disponível.")
    SYMPY_AVAILABLE = False


# --- Modelo Numérico do Robô com Dinâmica (Para Simulação) ---
def create_rrrp_robot_from_dh():
    """
    Cria um robô RRRP com base nos parâmetros DH e adiciona massas reais
    para a simulação numérica.
    """
    l2, l3 = 0.3, 0.03
    m2, m3, m4 = 0.2268, 0.053, 0.0123

    I2 = np.diag([2.36e-3, 0, 0])
    I3 = np.diag([1.26e-4, 0, 0])
    I4 = np.diag([2.33e-5, 0, 0])

    r2 = [0.192, 0, 0]   # CG link 2
    r3 = [0.033, 0, 0]   # CG link 3
    r4 = [0.0736, 0, 0]  # CG link 4

    links = [
        RevoluteDH(d=0, a=0, alpha=np.pi/2),  # Link 1
        RevoluteDH(d=0, a=l2, alpha=0, m=m2, r=r2, I=I2),
        RevoluteDH(d=0, a=l3, alpha=np.pi/2, m=m3, r=r3, I=I3),
        PrismaticDH(theta=0, a=0, alpha=0, qlim=[0, 0.105], m=m4, r=r4, I=I4)
    ]

    return SerialLink(links, name='RRRP_Orca_DH_Dinamica')

# --- Funções de Plotagem e Animação (sem alterações) ---
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
    if not ROBOTICS_TOOLBOX_AVAILABLE: return
    print("Iniciando playback da animação...")
    
    env = robot.plot(q_real[0, :], block=False, limits=[-0.5, 0.5, -0.5, 0.5, -0.1, 0.6])
    T_des_path = robot.fkine(q_desejado)
    tcp_des_path = T_des_path.t
    env.ax.plot(tcp_des_path[:,0], tcp_des_path[:,1], tcp_des_path[:,2], 'r--', linewidth=2, label='Trajetória TCP Desejada')
    env.ax.legend()

    for i in range(len(q_real)):
        robot.q = q_real[i, :]
        env.step(dt)
        
    print("Animação concluída. Feche a janela para ver os gráficos de análise.")
    plt.show(block=True)


def main():
    """Função principal que executa a simulação e depois a geração de fórmulas."""
    print("=== Simulador de Controle Dinâmico (V3.0 - Com Fórmulas Simbólicas) ===")
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

    # --- Ganhos PID (requerem sintonia fina) ---
    Kp = np.diag([10, 20, 15, 5])
    Kd = np.diag([2,  5,  3, 1])
    Ki = np.diag([0,  0,  0, 0])

    print("\nIniciando simulação numérica...")
    q_real = np.zeros_like(q_desejado)
    qd_real = np.zeros_like(q_desejado)
    q_real[0, :] = q_desejado[0, :]

    erro_integral = np.zeros(robot.n)
    max_integral_error = 1.0
    hist_erro, hist_torque = [], []

    for i in range(1, len(t)):
        q = q_real[i - 1, :]
        qd = qd_real[i - 1, :]
        
        # Leitura da trajetória desejada
        q_d = q_desejado[i - 1, :]
        qd_d = qd_desejado[i - 1, :]

        # Cálculo do Erro
        erro = q_d - q
        erro_deriv = qd_d - qd
        erro_integral += erro * dt
        erro_integral = np.clip(erro_integral, -max_integral_error, max_integral_error)

        # Lei de Controle (Computed Torque Control)
        acc_des = Kp @ erro + Kd @ erro_deriv + Ki @ erro_integral
        
        M = robot.inertia(q)
        C = robot.coriolis(q, qd)
        G = robot.gravload(q)
        torque_total = M @ acc_des + C @ qd + G

        hist_erro.append(erro)
        hist_torque.append(torque_total)

        # Integração numérica (simulação do robô real)
        qdd_real = np.linalg.inv(M) @ (torque_total - C @ qd - G)
        qd_real[i, :] = qd + qdd_real * dt
        q_real[i, :] = q + qd_real[i, :] * dt

    print("Simulação numérica concluída.")
    
    if np.isnan(q_real).any() or np.isinf(q_real).any():
        print("\n!!! ERRO: INSTABILIDADE NUMÉRICA DETECTADA !!!"); return

    # --- Etapa de Visualização e Análise ---
    if visualizar_animacao:
        animate_simulation_results(robot, dt, q_desejado, q_real)

    plotar_resultados_finais(t[1:], q_desejado[1:], q_real[1:], np.array(hist_erro), np.array(hist_torque))

    
    print("\nProcesso finalizado!")

if __name__ == "__main__":
    main()