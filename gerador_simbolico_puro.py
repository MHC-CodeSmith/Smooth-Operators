#!/usr/bin/env python3
"""
Exibidor de Dinâmica Simbólica Genérica para o Robô RRRP (com Inércias)

Descrição:
Este script deriva as equações completas da dinâmica (translação + rotação)
de um robô RRRP de forma totalmente simbólica. Os resultados são então
simplificados para o caso em que a primeira junta está travada em θ₁=0 e
exibidos como texto simples no terminal.
"""
import sympy as sp
import time

def dh_matrix(theta, d, a, alpha):
    """Cria uma matriz de transformação de Denavit-Hartenberg simbólica."""
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [            0,               sp.sin(alpha),                sp.cos(alpha),               d],
        [            0,                           0,                            0,               1]
    ])

def main():
    print("="*80)
    print("INICIANDO DERIVAÇÃO SIMBÓLICA GENÉRICA PARA O ROBÔ RRRP (com Inércias)")
    print("="*80)
    start_time = time.time()

    # --- 1. DEFINIÇÕES TOTALMENTE SIMBÓLICAS ---
    print("\n[Passo 1/8] Definindo variáveis e parâmetros simbólicos genéricos...")

    t = sp.symbols('t')
    q1, q2, q3, d4 = [f(t) for f in sp.symbols('θ₁, θ₂, θ₃, d₄', cls=sp.Function)]
    
    q = sp.Matrix([q1, q2, q3, d4])
    qd = q.diff(t)
    qdd = qd.diff(t)

    # Parâmetros Físicos Genéricos
    m2, m3, m4 = sp.symbols('m₂, m₃, m₄')
    l2, l3 = sp.symbols('L₂, L₃')
    rc2, rc3, rc4 = sp.symbols('r_c2, r_c3, r_c4')
    g = sp.symbols('g')

    # Tensores de Inércia
    Ixx2, Iyy2, Izz2 = sp.symbols('I_xx2, I_yy2, I_zz2')
    Ixx3, Iyy3, Izz3 = sp.symbols('I_xx3, I_yy3, I_zz3')
    Ixx4, Iyy4, Izz4 = sp.symbols('I_xx4, I_yy4, I_zz4')

    I2 = sp.diag(Ixx2, Iyy2, Izz2)
    I3 = sp.diag(Ixx3, Iyy3, Izz3)
    I4 = sp.diag(Ixx4, Iyy4, Izz4)
    
    # --- 2. CINEMÁTICA (POSIÇÃO) ---
    print("[Passo 2/8] Calculando a cinemática direta...")
    
    T01 = dh_matrix(q1, 0, 0, sp.pi/2)
    T12 = dh_matrix(q2, 0, l2, 0)
    T23 = dh_matrix(q3, 0, l3, sp.pi/2)
    T34 = dh_matrix(0, d4, 0, 0)
    
    T = [T01, T01*T12, T01*T12*T23, T01*T12*T23*T34]
    
    # --- 3. CINEMÁTICA (VELOCIDADES) ---
    print("[Passo 3/8] Calculando velocidades lineares e angulares...")

    P_c = [(T[i] * sp.Matrix([rc, 0, 0, 1]))[:3,:] for i, rc in enumerate([0, rc2, rc3, rc4])][1:]
    V_c = [pc.diff(t) for pc in P_c]

    z_unit = sp.Matrix([0,0,1])
    w = [sp.zeros(3,1)] * 5
    w[1] = T01[:3,:3] * z_unit * qd[0]
    w[2] = w[1] + T[1][:3,:3] * z_unit * qd[1]
    w[3] = w[2] + T[2][:3,:3] * z_unit * qd[2]
    w[4] = w[3]

    # --- 4. CÁLCULO DAS ENERGIAS ---
    print("[Passo 4/8] Calculando as energias Cinética e Potencial...")

    K_t = (0.5 * m2 * (V_c[0].T * V_c[0])[0] +
           0.5 * m3 * (V_c[1].T * V_c[1])[0] +
           0.5 * m4 * (V_c[2].T * V_c[2])[0])

    K_r = (0.5 * (w[2].T * T[1][:3,:3] * I2 * T[1][:3,:3].T * w[2])[0] +
           0.5 * (w[3].T * T[2][:3,:3] * I3 * T[2][:3,:3].T * w[3])[0] +
           0.5 * (w[4].T * T[3][:3,:3] * I4 * T[3][:3,:3].T * w[4])[0])
           
    K = K_t + K_r
    P = (m2 * g * P_c[0][2] + m3 * g * P_c[1][2] + m4 * g * P_c[2][2])
    
    # --- 5. DERIVAÇÃO DAS MATRIZES DA DINÂMICA ---
    print("[Passo 5/8] Derivando as matrizes M(q) e G(q)...")
    M = sp.Matrix([[sp.simplify(sp.diff(K, qdi, qdj)) for qdj in qd] for qdi in qd])
    G = sp.Matrix([P.diff(qi) for qi in q])
    
    # --- 6. CÁLCULO DA MATRIZ DE CORIOLIS C(q, qd) ---
    print("[Passo 6/8] Calculando a matriz C(q, qd)...")
    print("           (Esta etapa é MUITO demorada, pode levar vários minutos)")
    n = len(q)
    C = sp.zeros(n, n)
    for k in range(n):
        for j in range(n):
            c_sum = 0
            for i in range(n):
                christoffel = 0.5 * (M[k, j].diff(q[i]) + M[k, i].diff(q[j]) - M[i, j].diff(q[k]))
                c_sum += christoffel * qd[i]
            C[k, j] = c_sum
            
    # --- 7. MONTAGEM FINAL ---
    print("\n[Passo 7/8] Montando e simplificando a equação final...")
    M_s = sp.trigsimp(M)
    C_s = sp.trigsimp(C)
    G_s = sp.trigsimp(G)
    tau = M_s * qdd + C_s * qd + G_s
    
    # --- 8. EXIBIÇÃO NO TERMINAL ---
    print("\n[Passo 8/8] Exibindo fórmulas simplificadas para θ₁=0...")
    
    q_syms, qd_syms, qdd_syms = sp.symbols('θ₁,θ₂,θ₃,d₄'), sp.symbols('θ̇₁,θ̇₂,θ̇₃,ḋ₄'), sp.symbols('θ̈₁,θ̈₂,θ̈₃,d̈₄')
    
    # Dicionário para converter as funções f(t) em símbolos estáticos
    subs_generic = {**{q[i]: q_syms[i] for i in range(n)}, 
                    **{qd[i]: qd_syms[i] for i in range(n)}, 
                    **{qdd[i]: qdd_syms[i] for i in range(n)}}
                    
    # Dicionário para aplicar a simplificação θ₁=0, θ̇₁=0, θ̈₁=0
    subs_simplify = {q_syms[0]: 0, qd_syms[0]: 0, qdd_syms[0]: 0}

    # Função auxiliar para aplicar as substituições
    def substitute_and_simplify(expr):
        return sp.trigsimp(expr.subs(subs_generic).subs(subs_simplify))

    print("\n\n" + "="*80)
    print(" FÓRMULAS DA DINÂMICA (Simplificadas para θ₁=0, θ̇₁=0, θ̈₁=0)")
    print("="*80 + "\n")
    
    print("MATRIZ DE INÉRCIA M(q):\n")
    print(str(substitute_and_simplify(M_s)))
    print("\n\n" + "="*80 + "\n")

    print("MATRIZ DE CORIOLIS E FORÇAS CENTRÍPETAS C(q, q̇):\n")
    print(str(substitute_and_simplify(C_s)))
    print("\n\n" + "="*80 + "\n")

    print("VETOR DE FORÇAS GRAVITACIONAIS G(q):\n")
    print(str(substitute_and_simplify(G_s)))
    print("\n\n" + "="*80 + "\n")

    print("EQUAÇÕES FINAIS DO TORQUE/FORÇA τ = Mq̈ + Cq̇ + G:\n")
    for i in range(n):
        print("-" * 80)
        print(f"Torque/Força para a Junta {i+1} (τ[{i+1}])")
        print("-" * 80)
        print(str(substitute_and_simplify(tau[i])))
        print("\n")

    end_time = time.time()
    print("="*80)
    print(f"Tempo total de cálculo: {end_time - start_time:.2f} segundos.")
    print("="*80)

if __name__ == "__main__":
    main()