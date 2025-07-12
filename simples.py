#!/usr/bin/env python3
"""
Simulação do Robô RRRP - controller_simulation.py (Versão 3.3 - Plots Corrigidos)

Autor: André MK022 (Cuca) - Adaptado por Gemini com base em novo modelo
Descrição: Corrige a plotagem para iniciar em t=0 e remove a visualização
           da Junta 1 (travada) para focar nas juntas ativas.
"""
import numpy as np
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
import time
# ... outras importações
from sklearn.metrics import mean_squared_error, r2_score
from scipy.stats import pearsonr
# ... outras importações
import pandas as pd
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

# --- Funções de Plotagem e Animação ---
def calcular_metricas_performance(t, q_des, q_real, erro):
    num_juntas = q_des.shape[1]
    metricas = {}
    
    for i in range(num_juntas):
        if i == 0: # Ignora a Junta 1 travada
            metricas[f'junta_{i+1}'] = {'status': 'Travada (ganhos zero)'}
            continue

        # Sobressinal
        max_des = np.max(q_des[:, i])
        min_des = np.min(q_des[:, i])
        pico_real = np.max(q_real[:, i]) if max_des > 0 else np.min(q_real[:, i])
        amplitude_des = max_des - min_des
        
        sobressinal = 0
        if amplitude_des > 1e-9: # Aumentada a tolerância para evitar divisão por zero
            sobressinal = ((pico_real - max_des) / amplitude_des) * 100

        # Erro de regime (últimos 20% da simulação)
        n_final = int(0.2 * len(erro))
        erro_regime = np.mean(np.abs(erro[-n_final:, i]))
        
        # Tempo de estabilização (quando erro fica < 2% da amplitude do movimento)
        try:
            threshold = 0.02 * amplitude_des if amplitude_des > 1e-9 else 0.001
            indices_fora_faixa = np.where(np.abs(erro[:, i]) > threshold)[0]
            ultimo_fora_da_faixa = indices_fora_faixa[-1] if len(indices_fora_faixa) > 0 else 0
            tempo_estab = t[ultimo_fora_da_faixa]
        except IndexError:
            tempo_estab = t[-1] # Se nunca estabilizar, retorna o tempo total

        metricas[f'junta_{i+1}'] = {
            'sobressinal_%': max(0, sobressinal),
            'erro_regime': erro_regime,
            'tempo_estab_s': tempo_estab
        }
    return metricas

def analise_avancada_de_erro(t, erro_total):
    """
    Realiza uma análise avançada do erro de controle ao longo do tempo,
    gerando um painel de diagnósticos para cada junta ativa com cores de alto contraste.
    """
    num_juntas = erro_total.shape[1]
    titulos_juntas = ['Junta 1 (θ₁)', 'Junta 2 (θ₂)', 'Junta 3 (θ₃)', 'Junta 4 (d₄)']

    print("\n" + "="*40)
    print(" ANÁLISE AVANÇADA DE ERRO POR JUNTA")
    print("="*40)

    for i in range(1, num_juntas): # Pula a Junta 1
        erro_junta = erro_total[:, i]

        # --- Cálculos Estatísticos Gerais ---
        mean_err = np.mean(erro_junta)
        std_err = np.std(erro_junta)
        max_err = np.max(erro_junta)
        min_err = np.min(erro_junta)

        print(f"\n--- Estatísticas para {titulos_juntas[i].upper()} ---")
        print(f"  - Erro Médio:         {mean_err:.6f}")
        print(f"  - Desvio Padrão (σ):  {std_err:.6f}")
        print(f"  - Erro Máximo:        {max_err:.6f}")
        print(f"  - Erro Mínimo:        {min_err:.6f}")

        # --- Cálculos de Janela Móvel ---
        window_size = 50
        df = pd.DataFrame(erro_junta)
        moving_avg = df.rolling(window=window_size, min_periods=1).mean()
        moving_std = df.rolling(window=window_size, min_periods=1).std()

        # --- Geração do Painel de Gráficos ---
        fig, axs = plt.subplots(2, 2, figsize=(18, 10))
        fig.suptitle(f'Painel de Análise de Erro - {titulos_juntas[i]}', fontsize=18)

        # 1. Erro vs. Tempo com Bandas de Desvio Padrão (CORES MELHORADAS)
        ax1 = axs[0, 0]
        ax1.plot(t, erro_junta, color='limegreen', linewidth=2.0, label='Erro Instantâneo') # << COR E ESPESSURA
        ax1.axhline(mean_err, color='crimson', linestyle='--', linewidth=1.5, label=f'Média ({mean_err:.3f})') # << COR E ESPESSURA
        ax1.axhline(mean_err + std_err, color='darkorange', linestyle='-.', linewidth=1.5, label=f'Média + 1σ ({std_err:.3f})') # << COR, ESTILO E ESPESSURA
        ax1.axhline(mean_err - std_err, color='darkorange', linestyle='-.', linewidth=1.5, label='Média - 1σ') # << COR, ESTILO E ESPESSURA
        ax1.set_title('Erro ao Longo do Tempo e Desvio Padrão')
        ax1.set_xlabel('Tempo (s)')
        ax1.set_ylabel('Erro')
        ax1.legend()
        ax1.grid(True, linestyle=':')

        # 2. Histograma da Distribuição do Erro
        ax2 = axs[0, 1]
        ax2.hist(erro_junta, bins=50, alpha=0.75, color='mediumblue') # << COR
        ax2.axvline(mean_err, color='crimson', linestyle='--', linewidth=1.5, label=f'Média ({mean_err:.3f})') # << COR E ESPESSURA
        ax2.set_title('Distribuição do Erro (Histograma)')
        ax2.set_xlabel('Valor do Erro')
        ax2.set_ylabel('Frequência')
        ax2.legend()
        ax2.grid(True, linestyle=':')

        # 3. Média Móvel do Erro (VISIBILIDADE MELHORADA)
        ax3 = axs[1, 0]
        ax3.plot(t, erro_junta, color='lightgray', linewidth=1.5, label='Erro Original') # << COR E ESPESSURA
        ax3.plot(t, moving_avg, color='blue', linewidth=2.5, label=f'Média Móvel (Janela={window_size})') # << COR E ESPESSURA
        ax3.set_title('Tendência do Erro (Média Móvel)')
        ax3.set_xlabel('Tempo (s)')
        ax3.set_ylabel('Erro')
        ax3.legend()
        ax3.grid(True, linestyle=':')

        # 4. Desvio Padrão Móvel
        ax4 = axs[1, 1]
        ax4.plot(t, moving_std, color='darkviolet', linewidth=2, label=f'Desvio Padrão Móvel (Janela={window_size})') # << COR
        ax4.set_title('Variabilidade do Erro (Desvio Padrão Móvel)')
        ax4.set_xlabel('Tempo (s)')
        ax4.set_ylabel('Desvio Padrão (σ)')
        ax4.legend()
        ax4.grid(True, linestyle=':')

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show(block=True)

def plotar_trajetoria_endeffector_xz(robot, q_des, q_real):
    """
    Calcula a cinemática direta e plota a trajetória do end-effector (TCP)
    desejada vs. a real no plano XZ.
    """
    print("\n" + "="*45)
    print(" ANÁLISE DA TRAJETÓRIA DO END-EFFECTOR (PLANO XZ)")
    print("="*45)

    # 1. Calcular a cinemática direta para toda a trajetória
    # Isso converte o ângulo de cada junta em uma posição (x,y,z) do end-effector
    T_desejado = robot.fkine(q_des)
    T_real = robot.fkine(q_real)

    # 2. Extrair as coordenadas de translação (x, y, z)
    caminho_desejado_xyz = T_desejado.t
    caminho_real_xyz = T_real.t

    # --- Cálculo do Erro Cartesiano ---
    erro_cartesiano = np.linalg.norm(caminho_desejado_xyz - caminho_real_xyz, axis=1)
    erro_medio_cartesiano = np.mean(erro_cartesiano)
    erro_max_cartesiano = np.max(erro_cartesiano)
    print(f"  - Erro Cartesiano Médio (distância): {erro_medio_cartesiano*1000:.3f} mm")
    print(f"  - Erro Cartesiano Máximo (distância): {erro_max_cartesiano*1000:.3f} mm")


    # --- Geração do Gráfico 2D (Plano XZ) ---
    plt.figure(figsize=(12, 10))
    
    # Plot da trajetória desejada
    plt.plot(caminho_desejado_xyz[:, 0], caminho_desejado_xyz[:, 2], 
             color='crimson', linestyle='--', linewidth=3, label='TCP Desejado')
    
    # Plot da trajetória real
    plt.plot(caminho_real_xyz[:, 0], caminho_real_xyz[:, 2], 
             color='deepskyblue', linestyle='-', linewidth=2.5, label='TCP Real')

    # Marcar pontos de início e fim
    plt.scatter(caminho_desejado_xyz[0, 0], caminho_desejado_xyz[0, 2], s=100, c='red', marker='o', label='Início', zorder=5)
    plt.scatter(caminho_desejado_xyz[-1, 0], caminho_desejado_xyz[-1, 2], s=100, c='black', marker='x', label='Fim', zorder=5)

    plt.title('Trajetória do End-Effector no Plano XZ', fontsize=18)
    plt.xlabel('Eixo X (m)', fontsize=12)
    plt.ylabel('Eixo Z (m)', fontsize=12)
    plt.legend(fontsize=12)
    plt.grid(True, linestyle=':')
    
    # ESSENCIAL: Garante que a escala dos eixos X e Z seja a mesma,
    # para que um círculo não pareça uma elipse.
    plt.axis('equal') 
    
    plt.show(block=True)

def plotar_resultados_finais(t, q_des, q_real, erro, torque):
    num_juntas = q_des.shape[1]
    titulos_juntas = ['Junta 1 (θ₁)', 'Junta 2 (θ₂)', 'Junta 3 (θ₃)', 'Junta 4 (d₄)']
    unidades = ['rad', 'rad', 'rad', 'm']

    # --- Figura 1: Posição ---
    fig1, axs1 = plt.subplots(num_juntas - 1, 1, figsize=(15, 8), sharex=True)
    fig1.suptitle('Seguimento de Trajetória', fontsize=16)
    for i in range(1, num_juntas):
        ax = axs1[i-1]
        # Usando um vermelho vivo e um azul-celeste para alto contraste
        ax.plot(t, q_des[:, i], color='crimson', linestyle='--', label='Desejado', linewidth=2.5) ## << MUDANÇA
        ax.plot(t, q_real[:, i], color='deepskyblue', linestyle='-', label='Obtido', linewidth=2)    ## << MUDANÇA
        ax.set_title(titulos_juntas[i])
        ax.set_ylabel(f'Posição ({unidades[i]})')
        ax.grid(True, linestyle=':', alpha=0.7)
        ax.legend()
    plt.xlabel('Tempo (s)', fontsize=12)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show(block=True)

    # --- Figura 2: Erro ---
    fig2, axs2 = plt.subplots(num_juntas - 1, 1, figsize=(15, 8), sharex=True)
    fig2.suptitle('Erro de Seguimento', fontsize=16)
    for i in range(1, num_juntas):
        ax = axs2[i-1]
        # Laranja escuro é excelente para destacar o erro contra o fundo branco
        ax.plot(t, erro[:, i], color='darkorange', linewidth=2) ## << MUDANÇA
        ax.set_title(titulos_juntas[i])
        ax.set_ylabel(f'Erro ({unidades[i]})')
        ax.grid(True, linestyle=':', alpha=0.7)
    plt.xlabel('Tempo (s)', fontsize=12)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show(block=True)

    # --- Figura 3: Torque ---
    fig3, axs3 = plt.subplots(num_juntas - 1, 1, figsize=(15, 8), sharex=True)
    fig3.suptitle('Torque Aplicado', fontsize=16)
    for i in range(1, num_juntas):
        ax = axs3[i-1]
        # Um roxo/violeta forte se destaca bem
        ax.plot(t, torque[:, i], color='darkviolet', linewidth=2) ## << MUDANÇA
        ax.set_title(titulos_juntas[i])
        ax.set_ylabel('Torque/Força (N.m/N)')
        ax.grid(True, linestyle=':', alpha=0.7)
    plt.xlabel('Tempo (s)', fontsize=12)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show(block=True)

    # --- Métricas ---
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
    
    for q in q_real[::5, :]:
        robot.q = q
        env.step(dt*5)
    
    print("Animação concluída. Feche a janela para ver os gráficos de análise.")
    plt.show(block=True)

def main():
    print("=== Simulador de Controle Dinâmico (V3.3 - Plots Corrigidos) ===")
    visualizar_animacao = True
    if visualizar_animacao and not ROBOTICS_TOOLBOX_AVAILABLE:
        print("AVISO: Robotics Toolbox não encontrado. A animação visual será pulada.")
        visualizar_animacao = False

    robot = create_rrrp_robot_from_dh()

    try:
        q_desejado = np.load('trajetoria_desejada.npy')
        print(f"Trajetória com {len(q_desejado)} pontos carregada com sucesso.")
        q_desejado[:, 0] = 0
    except FileNotFoundError:
        print("ERRO CRÍTICO: Arquivo 'trajetoria_desejada.npy' não encontrado.")
        return

    dt = 0.01
    t = np.arange(0, len(q_desejado) * dt, dt)
    qd_desejado = np.gradient(q_desejado, dt, axis=0)
    qdd_desejado = np.gradient(qd_desejado, dt, axis=0)

    # --- CONFIGURAÇÃO DOS GANHOS DE CONTROLE ---
    # Perfil: OVERCLOCK EXTREMO (Teste de Limites)
    kp_gains = [0, 400, 400, 200]
    kd_gains = [0, 60, 50, 40]
    ki_gains = [0, 70, 45, 40]
    
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
    
    # Inicializa histórico com o estado em t=0 (erro inicial, torque zero)
    erro_inicial = q_desejado[0] - q_real[0]
    hist_erro, hist_torque = [erro_inicial], [np.zeros(robot.n)]

    start_time = time.time()
    for i in range(1, len(t)):
        q = q_real[i-1]
        qd = qd_real[i-1]
        erro = q_desejado[i] - q
        erro_deriv = qd_desejado[i] - qd
        erro_integral += erro * dt
        erro_integral = np.clip(erro_integral, -2.0, 2.0)
        
        acc_des_compensada = Kp @ erro + Kd @ erro_deriv + Ki @ erro_integral
        
        M = robot.inertia(q)
        C = robot.coriolis(q, qd)
        G = robot.gravload(q)
        
        torque_total = M @ acc_des_compensada + C @ qd + G
        
        hist_erro.append(erro)
        hist_torque.append(torque_total)
        
        qdd_real = np.linalg.inv(M) @ (torque_total - C @ qd - G)
        
        qd_real[i] = qd + qdd_real * dt
        q_real[i] = q + qd * dt
        q_real[i, 0] = 0.0
        qd_real[i, 0] = 0.0
        
    end_time = time.time()
    print(f"Simulação numérica concluída em {end_time - start_time:.2f} segundos.")

    if np.isnan(q_real).any() or np.isinf(q_real).any():
        print("\n" + "!"*54)
        print("!!! ERRO: INSTABILIDADE NUMÉRICA DETECTADA !!!")
        print("!!! Ganhos Kp provavelmente muito altos. O sistema se tornou instável.")
        print("!"*54 + "\n")
        return

    if visualizar_animacao:
        animate_simulation_results(robot, dt, q_desejado, q_real)

    # Passa os arrays completos para a função de plotagem
    plotar_resultados_finais(t, q_desejado, q_real,
                             np.array(hist_erro), np.array(hist_torque))
    # NOVA CHAMADA PARA ANÁLISE AVANÇADA DE ERRO
    analise_avancada_de_erro(t, np.array(hist_erro))
    
    plotar_trajetoria_endeffector_xz(robot, q_desejado, q_real)

    
    print("\nProcesso finalizado!")

if __name__ == "__main__":
    main()