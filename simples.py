#!/usr/bin/env python3
"""
Simulação do Robô RRRP - controller_simulation.py (Versão 3.2 - Sintonia de Ganhos)

Autor: André MK022 (Cuca) - Adaptado por Gemini com base em novo modelo
Descrição: Versão focada na sintonia de ganhos PID. Trava a junta 1 (theta_1)
           zerando seus ganhos e fornece uma estrutura clara para otimizar
           a performance das outras juntas.
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

# --- Modelo Numérico do Robô com Dinâmica (Para Simulação) ---
def create_rrrp_robot_from_dh():
    # Parâmetros Físicos do Robô Orca (aproximados)
    l2, l3 = 0.3, 0.03
    m2, m3, m4 = 0.2268, 0.053, 0.0123
    I2 = np.diag([2.36e-3, 0, 0])
    I3 = np.diag([1.26e-4, 0, 0])
    I4 = np.diag([2.33e-5, 0, 0])
    r2 = [0.192, 0, 0] # Centro de massa do elo 2
    r3 = [0.033, 0, 0] # Centro de massa do elo 3
    r4 = [0.0736, 0, 0]# Centro de massa do elo 4
    
    links = [
        RevoluteDH(d=0, a=0, alpha=np.pi/2),                     # Junta 1 (θ₁) - Rotação na base
        RevoluteDH(d=0, a=l2, alpha=0, m=m2, r=r2, I=I2),      # Junta 2 (θ₂)
        RevoluteDH(d=0, a=l3, alpha=np.pi/2, m=m3, r=r3, I=I3), # Junta 3 (θ₃)
        PrismaticDH(theta=0, a=0, alpha=0, qlim=[0, 0.105], m=m4, r=r4, I=I4) # Junta 4 (d₄) - Prismatic
    ]
    return SerialLink(links, name='RRRP_Orca_Sintonia')

# --- Funções de Plotagem e Animação (sem alterações) ---
def calcular_metricas_performance(t, q_des, q_real, erro):
    num_juntas = q_des.shape[1]
    metricas = {}
    
    for i in range(num_juntas):
        # Ignora métricas para juntas travadas (ganho zero)
        if np.max(np.abs(q_des[:, i])) == 0 and np.max(np.abs(q_real[:, i])) < 1e-6:
             metricas[f'junta_{i+1}'] = {'status': 'Travada (ganhos zero)'}
             continue

        # Sobressinal
        max_des = np.max(q_des[:, i])
        min_des = np.min(q_des[:, i])
        pico_real = np.max(q_real[:, i]) if max_des > 0 else np.min(q_real[:, i])
        amplitude_des = max_des - min_des
        
        sobressinal = 0
        if amplitude_des > 1e-6:
            if max_des > 0:
                sobressinal = ((pico_real - max_des) / amplitude_des) * 100
            else: # Movimento negativo
                sobressinal = ((pico_real - min_des) / amplitude_des) * 100

        # Erro de regime (últimos 20% da simulação)
        n_final = int(0.2 * len(erro))
        erro_regime = np.mean(np.abs(erro[-n_final:, i]))
        
        # Tempo de estabilização (quando erro fica < 2% da amplitude do movimento)
        try:
            threshold = 0.02 * amplitude_des if amplitude_des > 1e-6 else 0.001
            indices_fora_faixa = np.where(np.abs(erro[:, i]) > threshold)[0]
            ultimo_fora_da_faixa = indices_fora_faixa[-1] if len(indices_fora_faixa) > 0 else 0
            tempo_estab = t[ultimo_fora_da_faixa]
        except:
            tempo_estab = t[-1] # Se nunca estabilizar, retorna o tempo total

        metricas[f'junta_{i+1}'] = {
            'sobressinal_%': max(0, sobressinal), # Evita sobressinal negativo se não houver
            'erro_regime': erro_regime,
            'tempo_estab_s': tempo_estab
        }
    return metricas

def plotar_resultados_finais(t, q_des, q_real, erro, torque):
    num_juntas = q_des.shape[1]
    titulos_juntas = ['Junta 1 (θ₁)', 'Junta 2 (θ₂)', 'Junta 3 (θ₃)', 'Junta 4 (d₄)']
    unidades = ['rad', 'rad', 'rad', 'm']
    
    plt.figure(figsize=(15, 10))
    for i in range(num_juntas):
        plt.subplot(num_juntas, 1, i+1)
        plt.plot(t, q_des[:, i], 'r--', label='Desejado', linewidth=2)
        plt.plot(t, q_real[:, i], 'b-', label='Obtido', linewidth=1.5)
        plt.title(f'Seguimento de Trajetória - {titulos_juntas[i]}')
        plt.ylabel(f'Posição ({unidades[i]})')
        plt.grid(True, linestyle=':', alpha=0.6)
        plt.legend()
    plt.xlabel('Tempo (s)')
    plt.tight_layout()
    plt.show(block=True)

    plt.figure(figsize=(15, 8))
    for i in range(num_juntas):
        plt.subplot(2, 2, i+1)
        plt.plot(t, erro[:, i], 'g-', linewidth=1.5)
        plt.title(f'Erro de Seguimento - {titulos_juntas[i]}')
        plt.ylabel(f'Erro ({unidades[i]})')
        plt.xlabel('Tempo (s)')
        plt.grid(True, linestyle=':', alpha=0.6)
    plt.tight_layout()
    plt.show(block=True)

    plt.figure(figsize=(15, 8))
    for i in range(num_juntas):
        plt.subplot(2, 2, i+1)
        plt.plot(t, torque[:, i], 'm-', linewidth=1.5)
        plt.title(f'Torque Aplicado - {titulos_juntas[i]}')
        plt.ylabel('Torque/Força (N.m/N)')
        plt.xlabel('Tempo (s)')
        plt.grid(True, linestyle=':', alpha=0.6)
    plt.tight_layout()
    plt.show(block=True)

    metricas = calcular_metricas_performance(t, q_des, q_real, erro)
    print("\n" + "="*25)
    print(" MÉTRICAS DE PERFORMANCE")
    print("="*25)
    for junta, metrica in metricas.items():
        print(f"\n{junta.upper()}:")
        if 'status' in metrica:
            print(f"  - Status: {metrica['status']}")
        else:
            print(f"  - Sobressinal (Overshoot): {metrica['sobressinal_%']:.2f}%")
            print(f"  - Erro de Regime Estacionário: {metrica['erro_regime']:.6f} ({unidades[int(junta[-1])-1]})")
            print(f"  - Tempo de Estabilização (2%): {metrica['tempo_estab_s']:.3f} s")

def animate_simulation_results(robot, dt, q_desejado, q_real):
    if not ROBOTICS_TOOLBOX_AVAILABLE:
        return
    print("\nIniciando playback da animação...")
    env = robot.plot(q_real[0, :], block=False, limits=[-0.5, 0.5, -0.5, 0.5, -0.1, 0.6])
    T_des_path = robot.fkine(q_desejado)
    tcp_des_path = T_des_path.t
    env.ax.plot(tcp_des_path[:,0], tcp_des_path[:,1], tcp_des_path[:,2], 'r--', linewidth=2, label='TCP Desejada')
    T_real_path = robot.fkine(q_real)
    tcp_real_path = T_real_path.t
    env.ax.plot(tcp_real_path[:,0], tcp_real_path[:,1], tcp_real_path[:,2], 'b:', linewidth=2.5, label='TCP Real')
    env.ax.legend()
    time.sleep(1)
    
    for q in q_real[::5, :]: # Plota a cada 5 frames para acelerar a animação
        robot.q = q
        env.step(dt*5)
    
    print("Animação concluída. Feche a janela para ver os gráficos de análise.")
    plt.show(block=True)

def main():
    print("=== Simulador de Controle Dinâmico (V3.2 - Sintonia de Ganhos) ===")
    visualizar_animacao = True
    if visualizar_animacao and not ROBOTICS_TOOLBOX_AVAILABLE:
        print("AVISO: Robotics Toolbox não encontrado. A animação visual será pulada.")
        visualizar_animacao = False

    robot = create_rrrp_robot_from_dh()

    try:
        q_desejado = np.load('trajetoria_desejada.npy')
        print(f"Trajetória com {len(q_desejado)} pontos carregada com sucesso.")
        # Garante que a trajetória desejada para a junta 1 seja zero.
        q_desejado[:, 0] = 0
    except FileNotFoundError:
        print("ERRO CRÍTICO: Arquivo 'trajetoria_desejada.npy' não encontrado.")
        print("Certifique-se que o arquivo de trajetória está na mesma pasta que este script.")
        return

    dt = 0.01
    t = np.arange(0, len(q_desejado) * dt, dt)
    # Calcula a velocidade desejada usando gradiente, mais robusto que np.diff
    qd_desejado = np.gradient(q_desejado, dt, axis=0)
    # Calcula a aceleração desejada (usada em alguns controladores, mas não aqui)
    qdd_desejado = np.gradient(qd_desejado, dt, axis=0)

    # --- CONFIGURAÇÃO DOS GANHOS DE CONTROLE (ÁREA DE SINTONIA) ---
    #
    # INSTRUÇÕES:
    # 1. Altere os valores abaixo para testar diferentes ganhos.
    # 2. A primeira junta (θ₁) está travada (ganhos = 0).
    # 3. Os ganhos são para [Junta1, Junta2, Junta3, Junta4] respectivamente.
    #
    # SUGESTÃO DE PARTIDA:
    # Kp: [0, 25, 20, 15]  -> Resposta rápida
    # Kd: [0, 10,  8,  5]  -> Amortecimento para reduzir sobressinal
    # Ki: [0,  1,  1,  0.5] -> Correção de erro de regime (use valores pequenos!)
# Perfil: OVERCLOCK (Velocidade Máxima Experimental)
# AVISO: Alto risco de sobressinal e oscilações. Monitore os gráficos com atenção.

    kp_gains = [0, 400, 400, 200]  # AUMENTO AGRESSIVO em Kp para todas as juntas.

    kd_gains = [0, 60, 50, 40]  # Aumento proporcional em Kd para tentar conter o sobressinal.

    ki_gains = [0, 70, 45, 40]  # Ki mantido para garantir o erro de regime baixo.
    Kp = np.diag(kp_gains)
    Kd = np.diag(kd_gains)
    Ki = np.diag(ki_gains)
    
    print("\n--- Ganhos PID Utilizados ---")
    print(f"Kp = {np.diag(Kp)}")
    print(f"Kd = {np.diag(Kd)}")
    print(f"Ki = {np.diag(Ki)}")
    print("-----------------------------\n")

    print("Iniciando simulação numérica...")
    q_real = np.zeros_like(q_desejado)
    qd_real = np.zeros_like(q_desejado)
    q_real[0] = q_desejado[0]
    erro_integral = np.zeros(robot.n)
    hist_erro, hist_torque = [], []

    start_time = time.time()
    for i in range(1, len(t)):
        q = q_real[i-1]
        qd = qd_real[i-1]

        # Erro de posição, velocidade e integral
        erro = q_desejado[i] - q
        erro_deriv = qd_desejado[i] - qd
        erro_integral += erro * dt
        
        # Anti-windup: Limita a acumulação do erro integral para evitar instabilidade
        erro_integral = np.clip(erro_integral, -2.0, 2.0)

        # Lei de Controle (Computed Torque Control com PID)
        # acc_des = qdd_desejado[i] + Kd @ erro_deriv + Kp @ erro + Ki @ erro_integral # Forma completa
        acc_des_compensada = Kp @ erro + Kd @ erro_deriv + Ki @ erro_integral

        # Dinâmica do Robô
        M = robot.inertia(q)
        C = robot.coriolis(q, qd)
        G = robot.gravload(q)

        # Torque/Força calculado
        torque_total = M @ acc_des_compensada + C @ qd + G

        # Armazenamento para plots
        hist_erro.append(erro)
        hist_torque.append(torque_total)

        # Simulação da Dinâmica Direta (o que o robô realmente faria)
        # A aceleração real é resultado do torque aplicado sobre a dinâmica atual
        qdd_real = np.linalg.inv(M) @ (torque_total - C @ qd - G)

        # Integração Numérica (Euler) para obter a nova velocidade e posição
        qd_real[i] = qd + qdd_real * dt
        q_real[i] = q + qd * dt # Usar a velocidade do passo anterior (qd) é mais estável
        q_real[i, 0] = 0.0
        qd_real[i, 0] = 0.0
    end_time = time.time()
    print(f"Simulação numérica concluída em {end_time - start_time:.2f} segundos.")

    if np.isnan(q_real).any() or np.isinf(q_real).any():
        print("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print("!!! ERRO: INSTABILIDADE NUMÉRICA DETECTADA !!!")
        print("!!! Verifique os ganhos. Ganhos muito altos (Kp, Ki)")
        print("!!! ou muito baixos (Kd) podem causar este problema.")
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        return

    if visualizar_animacao:
        animate_simulation_results(robot, dt, q_desejado, q_real)

    # O primeiro ponto é o inicial, então plotamos a partir do segundo
    plotar_resultados_finais(t[1:], q_desejado[1:], q_real[1:],
                             np.array(hist_erro), np.array(hist_torque))

    print("\nProcesso finalizado!")

if __name__ == "__main__":
    main()