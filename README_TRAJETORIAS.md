# Simulação de Robô RRRP para Soldagem - Trajetórias

Este projeto contém scripts Python que demonstram como um robô RRRP (3 juntas rotacionais + 1 prismática) pode seguir trajetórias precisas para aplicações de soldagem, compensando o consumo do eletrodo.

## 📁 Arquivos do Projeto

### Scripts Principais

1. **`rrrp_robot_enhanced.py`** - Versão aprimorada com visualização do vetor de referência
2. **`demonstracao_tcp_trajetoria.py`** - Demonstração detalhada de como o TCP segue o vetor
3. **`exemplo_ik_trajetoria_complexa.py`** - Exemplos de IK para trajetórias complexas
4. **`rrrp_robot_qt.py`** - Versão otimizada para Qt5Agg (resolução de problemas de visualização)

### Scripts de Suporte

5. **`test_qt_backend.py`** - Teste do backend Qt5Agg
6. **`requirements.txt`** - Dependências do projeto
7. **`SOLUCAO_VISUALIZACAO.md`** - Solução para problemas de visualização

## 🚀 Como Executar

### 1. Configuração do Ambiente

```bash
# Criar ambiente virtual
python3 -m venv venv
source venv/bin/activate

# Instalar dependências
pip install -r requirements.txt
```

### 2. Executar Scripts

```bash
# Script principal aprimorado
python rrrp_robot_enhanced.py

# Demonstração detalhada do TCP
python demonstracao_tcp_trajetoria.py

# Exemplos de IK para trajetórias complexas
python exemplo_ik_trajetoria_complexa.py
```

## 📊 Explicação dos Scripts

### 1. `rrrp_robot_enhanced.py`

**O que faz:**
- Simula o robô RRRP com trajetória de soldagem
- Visualiza o vetor de referência (linha vermelha pontilhada)
- Mostra como o TCP segue exatamente o vetor
- Inclui análise detalhada da trajetória
- Demonstra capacidades de IK

**Características:**
- ✅ Visualização do vetor de referência
- ✅ Análise passo a passo da trajetória
- ✅ Verificação de precisão
- ✅ Demonstração de IK para casos gerais

### 2. `demonstracao_tcp_trajetoria.py`

**O que faz:**
- Demonstra matematicamente como o TCP segue o vetor
- Mostra o mecanismo de compensação da junta prismática
- Visualizações detalhadas do processo
- Explicação do princípio matemático

**Características:**
- ✅ Análise matemática detalhada
- ✅ Visualização do mecanismo de compensação
- ✅ Gráficos explicativos
- ✅ Verificação numérica da precisão

### 3. `exemplo_ik_trajetoria_complexa.py`

**O que faz:**
- Demonstra IK para trajetórias complexas
- Exemplos: circular, espiral, senoidal
- Verificação de precisão das soluções
- Comparação entre trajetória desejada e real

**Características:**
- ✅ Trajetórias circulares
- ✅ Trajetórias espirais
- ✅ Trajetórias senoidais
- ✅ Verificação de precisão
- ✅ Múltiplos chutes iniciais para IK

## 🔧 Princípio de Funcionamento

### Trajetória Linear (Caso Simples)

Para uma trajetória linear em X:

1. **Definição:** `x_desejado = [0.2, 0.3, 0.4, 0.5]` (vetor de referência)

2. **Configuração das juntas:**
   - `q1 = 0` (fixo)
   - `q2` varia de 90° a 0° (braço desce)
   - `q3 = 90° - q2` (mantém orientação)
   - `q4 = x_desejado - l2*cos(q2)` (compensação)

3. **Resultado:** TCP segue exatamente o vetor de referência

### Trajetórias Complexas (IK)

Para trajetórias arbitrárias:

1. **Definição:** Pontos 3D `(x_i, y_i, z_i)` da trajetória

2. **Para cada ponto:**
   ```python
   pose = SE3(x_i, y_i, z_i) * orientação_desejada
   q_i = robot.ikine_LM(pose)
   ```

3. **Resultado:** TCP segue exatamente a curva desejada

## 📈 Visualizações

### Gráficos Gerados

1. **Trajetória 3D:**
   - Linha azul: trajetória real do TCP
   - Linha vermelha pontilhada: vetor de referência
   - Pontos verdes/vermelhos: início/fim

2. **Análise de Compensação:**
   - Variação das juntas rotacionais
   - Compensação da junta prismática
   - Mecanismo matemático

3. **Comparação IK:**
   - Trajetória desejada vs real
   - Verificação de precisão
   - Taxa de sucesso do IK

## 🎯 Aplicações Práticas

### Soldagem Industrial

- **Compensação de eletrodo:** A junta prismática compensa o consumo
- **Trajetórias lineares:** Soldagem em linhas retas
- **Trajetórias complexas:** Soldagem em curvas e formas irregulares

### Outras Aplicações

- **Pintura:** Controle preciso de spray
- **Inspeção:** Movimento preciso para câmeras/sensores
- **Montagem:** Posicionamento preciso de componentes

## 🔍 Análise de Precisão

### Métricas

- **Erro 3D:** Distância entre posição desejada e real
- **Taxa de sucesso IK:** Porcentagem de pontos resolvidos
- **Erro médio:** Precisão geral da trajetória

### Padrões de Qualidade

- **Excelente:** Erro < 1mm
- **Boa:** Erro < 5mm
- **Aceitável:** Erro < 10mm

## 🛠️ Solução de Problemas

### Problemas de Visualização

Se encontrar erros de visualização:

1. **Verificar backend:**
   ```bash
   python test_qt_backend.py
   ```

2. **Instalar PyQt5:**
   ```bash
   sudo apt-get install python3-pyqt5
   ```

3. **Configurar DISPLAY:**
   ```bash
   export DISPLAY=:0
   ```

### Problemas de IK

Se o IK falhar:

1. **Ajustar chutes iniciais**
2. **Verificar limites das juntas**
3. **Reduzir complexidade da trajetória**
4. **Aumentar número de iterações**

## 📚 Referências

- **Robotics Toolbox for Python:** Documentação oficial
- **Cinemática de Robôs:** Teoria DH e transformações
- **Cinemática Inversa:** Métodos numéricos (Levenberg-Marquardt)

## 🤝 Contribuição

Para contribuir com o projeto:

1. Fork o repositório
2. Crie uma branch para sua feature
3. Commit suas mudanças
4. Push para a branch
5. Abra um Pull Request

## 📄 Licença

Este projeto está sob licença MIT. Veja o arquivo LICENSE para detalhes.

---

**Autor:** André MK022 (Cuca)  
**Descrição:** Simulação de robô RRRP para soldagem com compensação de eletrodo  
**Versão:** 2.0 - Versão Aprimorada com Visualizações 