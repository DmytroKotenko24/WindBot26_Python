# Bibliotecas necessárias
import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import os.path
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from tqdm import tqdm

script_dir = os.path.dirname(os.path.abspath(__file__))
porta_dir = os.path.join(script_dir, 'GeneratedFiles_OCVPorta_25')
torre_dir = os.path.join(script_dir, 'GeneratedFiles_OCVTorre_25')

image_rearenged = np.load(os.path.join(porta_dir, "arraySCANPorta.npy"))
image_rearengedT = np.load(os.path.join(torre_dir, "arraySCANTorre.npy"))
maior_contorno_bag = np.load(os.path.join(porta_dir, "maiorContornoPorta.npy"))
contorno_bag_image = np.load(os.path.join(porta_dir, "contorno_bag_image.npy"))
enablePlot=False

output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'GeneratedFiles_OCVCalculos_25')
os.makedirs(output_dir, exist_ok=True)

"""
# Configurações iniciais
image_rearenged = np.load("WindBot26/GeneratedFiles_OCVPorta_25/arraySCANPorta.npy")
image_rearengedT = np.load("WindBot26/GeneratedFiles_OCVTorre_25/arraySCANTorre.npy")
maior_contorno_bag=np.load("WindBot26/GeneratedFiles_OCVPorta_25/maiorContornoPorta.npy")
contorno_bag_image=np.load("WindBot26/GeneratedFiles_OCVPorta_25/contorno_bag_image.npy")
enablePlot=False
"""

"""Criacao pipeline"""
BAG_FILE = "Bag_Scan_Porta.bag"
# Cria pipeline para capturar dados da câmera
pipeline = rs.pipeline()
# Criar objeto de configuração para o pipeline
config = rs.config()
# Permite que o pipeline seja usado para reproduzir dados de um arquivo gravado (.bag)
rs.config.enable_device_from_file(config, BAG_FILE)
# Configurar o pipeline para transmitir o fluxo de profundidade da câmera
# Os parâmetros podem precisar ser ajustados conforme a resolução do arquivo .bag gravado
# enable_stream() configura o tipo de fluxo que será transmitido (neste caso e de 16 bits = Z16)
config.enable_stream(rs.stream.depth, rs.format.z16, 6) # 6 é a taxa de quadros (FPS) do fluxo de profundidade
# Iniciar o pipeline com a configuração, permite o processamento e transmissão de dados gravados
pipeline.start(config)

# Obtém os parâmetros intrínsecos da câmera
profile = pipeline.get_active_profile() # Obtêm o perfil de profundidade da câmera e outras câmeras se existirem
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()# Obtêm os parametros intrinsecos da câmera de profundidade


                                                                                            #(parâmetros essenciais para transformar as coordenadas 2D da imagem (pixeis) para coordenadas 3D
fx = intrinsics.fx  # Comprimento focal em pixeis para a direção x
fy = intrinsics.fy  # Comprimento focal em pixeis para a direção y
cx = intrinsics.ppx # Centro óptico da câmera X
cy = intrinsics.ppy # Centro ópticoda câmera Y
###############################################################################################################################################################################
###############################################################################################################################################################################
###############################################################################################################################################################################
###############################################################################################################################################################################
#               FUNÇOES#               FUNÇOES #               FUNÇOES #               FUNÇOES #               FUNÇOES #               FUNÇOES                       
###############################################################################################################################################################################
###############################################################################################################################################################################
###############################################################################################################################################################################
###############################################################################################################################################################################
###############################################################################################################################################################################
def pontoXYZ_para_matriz(x,y,z):
    matrizponto=np.eye(4)
    matrizponto[0,3]=x
    matrizponto[1,3]=y
    matrizponto[2,3]=z
    return matrizponto

def matriz_para_ponto(matriz):
    x=matriz[0,3]
    y=matriz[1,3]
    z=matriz[2,3]
    return x, y, z

def converter_ponto_para_referencial_origemMundo(ponto):
    x=ponto[0]
    y=ponto[1]
    z=ponto[2]
    matrizponto=pontoXYZ_para_matriz(x,y,z)
    matrizponto_corrigida_origem= matriz_p1_2_origemmundo @ matriz_camara2flange @ matriz_infravermelhoDireita2camara @ matrizponto
    ponto=matriz_para_ponto(matrizponto_corrigida_origem)
    return ponto

"""VEWR ESTA FUNCAO ABAIXO"""
def converter_ponto_para_referencial_P1(ponto):
    matrizponto_corrigida_origem=pontoXYZ_para_matriz(X,Y,Z)
    
    matrizpontoorigem = np.linalg.inv(matriz_infravermelhoDireita2camara) @ \
                        np.linalg.inv(matriz_camara2flange) @ \
                        np.linalg.inv(matriz_p1_2_origemmundo) @ \
                        matrizponto_corrigida_origem

    X,Y,Z=matriz_para_ponto(matrizpontoorigem)
    return ponto

# Função para converter pontos de pixel para coordenadas 3D
def pixel_para_3D(x, y,image_rearengedT,intrinsics): #Converte as coordenadas de um ponto em 2D na imagem de profundidade para coordenadas 3D no espaço real
    point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], image_rearengedT[y][x])
    X = point_3d[0]
    Y = point_3d[1]
    Z=point_3d[2]
    return (X, Y, Z) # Função retorna a coordenada 3D (X,Y,Z)

def ponto3D_para_pixel(X, Y, Z,intrinsics):
    pixel = rs.rs2_project_point_to_pixel(intrinsics, [X, Y, Z])
    x=pixel[0]
    y=pixel[1]
    return x, y

def pontoRealXYZ_para_pixel_referencial_P1(X, Y, Z):
    matrizponto_corrigida_origem=pontoXYZ_para_matriz(X,Y,Z)
    
    matrizpontoorigem = np.linalg.inv(matriz_infravermelhoDireita2camara) @ \
                        np.linalg.inv(matriz_camara2flange) @ \
                        np.linalg.inv(matriz_p1_2_origemmundo) @ \
                        matrizponto_corrigida_origem

    X,Y,Z=matriz_para_ponto(matrizpontoorigem)
    x_pixel,y_pixel=ponto3D_para_pixel(X,Y,Z,intrinsics)
    return x_pixel, y_pixel

#Função para calcular a distancia de um ponto 3D até uma reta 3D                                                        
def distancia_ponto_reta(arrayPontos, reta, dist_max):
    vx, vy, vz, x0, y0, z0 = reta.flatten()
    v = np.array([vx, vy, vz])
    p0 = np.array([x0, y0, z0])
    
    for ponto in arrayPontos:
        p = ponto  # (x, y, z)
        vetor = p - p0
        distancia = np.linalg.norm(np.cross(vetor, v)) / np.linalg.norm(v)
        
        if distancia > dist_max:
            return True
        return False

def ponto_mais_proximo_3D(pontoX, retaY):
    """pontoX: array-like de shape (3,) representando (x, y, z)
    retaY: array-like de shape (6,) ou (2, 3): [vx, vy, vz, x0, y0, z0] ou [[vx, vy, vz], [x0, y0, z0]]"""
    retaY = np.array(retaY)
    if retaY.shape == (2, 3):
        vx, vy, vz = retaY[0]
        x0, y0, z0 = retaY[1]
    else:
        vx, vy, vz, x0, y0, z0 = retaY.flatten()
    v = np.array([vx, vy, vz])
    p0 = np.array([x0, y0, z0])
    p = np.array(pontoX)
    t = np.dot(p - p0, v) / np.dot(v, v)
    proj = p0 + t * v
    return proj  # retorna ponto mais próximo na reta como array (x, y, z)

# Função para calcular o ponto mais próximo de uma reta a partir de um ponto dado                                                                
def ponto_mais_proximo(pontoX, retaY): #Divide os parâmetros da reta. "retaY" é um array com os parâmetros (vx, vy) e (x0, y0)
    vx, vy, x0, y0 = retaY.flatten()  # Parâmetros da reta: direções vx, vy e o ponto na reta (x0, y0)
    x_p, y_p = pontoX   # Coordenadas do ponto de entrada (pontoX)
    # Calcular o escalar t para projeção
    # A projeção do ponto sobre a reta é dada pela fórmula do escalar t, que determina a posição do ponto projetado sobre a reta.
    t = ((x_p - x0) * vx + (y_p - y0) * vy) / (vx**2 + vy**2)
    # Calcular coordenadas do ponto mais próximo na reta usando o escalar t
    # A projeção do ponto sobre a reta (ponto mais próximo) é dada pelas coordenadas x_proj e y_proj
    x_proj = x0 + t * vx
    y_proj = y0 + t * vy
    # Arredondamento das coordenadas do ponto projetado para inteiros (já que a imagem tem coordenadas inteiras)
    ponto=int(x_proj), int(y_proj) # Função que converte para inteiros
    # Converte as coordenadas do ponto para o formato de array
    ponto_convertido = [np.array([[ponto[0], ponto[1]]], dtype=np.int16)]
    #print("ponto ocnvertido=", ponto_convertido)
    # Retorna o ponto projetado na reta
    return (ponto_convertido)  # Retorna as coordenadas como um array de ponto projetado

def set_axes_equal(ax):
    """Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.
    Input
      ax: a matplotlib axis, e.g., as output from plt.gca()."""
    x_limits = ax.get_xlim3d()    
    y_limits = ax.get_ylim3d()    
    z_limits = ax.get_zlim3d()
    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)    
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)
    # The plot bounding box is a sphere in the sense of the infinity# norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])
    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
#############################################################################################
def normal_to_rpy(normal): 
    x, y = normal / np.linalg.norm(normal)  # Normalização do vetor dividindo as componentes x e y do vetor pela sua norma, trnasformando assim o vetor 'normal' em um vetor unitário
    # Cálculo dos ângulos
    z=0
    roll = np.arctan2(y, x) # Rotação em torno do eixo X (aponta com z)
    pitch = 0 # Rotação em torno do eixo Y
    yaw =  0  # Rotação em torno do eixo Z    
    return roll, pitch, yaw
#############################################################################################
def verifica_direcao(vetor, ponto_origem, centro_massa):
    # Calcula o vetor que aponta para o centro de massa
    vetor_cm = np.array(centro_massa) - np.array(ponto_origem)
    # Produto escalar entre o vetor e o vetor direção ao centro de massa
    produto_escalar = np.dot(vetor, vetor_cm)
    # Verifica a direção do vetor
    if produto_escalar >= 0:
        return True #O vetor aponta na direção do centro de massa
    elif produto_escalar < 0:
        return False #O vetor aponta no sentido oposto ao centro de massa
#############################################################################################
def calcular_normais(pontos_corrigidos_zs,centroMAssa): # Função que calcula as normais, com base na lista de pontos 3D onde cada ponto tem coordenadas (x,y,z) e o centro de massa (garantindo assim que as normais sejam orientadas corretamente)
    normais = [] # Armazena as normais calculadas para cada ponto
    rpy = [] #Armazena os ângulos de Euler (Roll, Pitch,Yaw)
    num_pontos = len(pontos_corrigidos_zs)
    for i in range(num_pontos): # Função precorre os pontos da lista 'pontos_corrigidos_zs' calculando a normal e os ângulos de Euler para cada ponto
	# Definição dos pontos atuais e vizinhos
        p_atual = np.array(pontos_corrigidos_zs[i][:2])  # Apenas X e Y
        if i==0:
            p_anterior = np.array(pontos_corrigidos_zs[len(pontos_corrigidos_zs)-1][:2])  
        else:
            p_anterior = np.array(pontos_corrigidos_zs[i - 1][:2])
        if i==len(pontos_corrigidos_zs)-1:
            p_proximo = np.array(pontos_corrigidos_zs[0][:2]) 
        else:    
            p_proximo = np.array(pontos_corrigidos_zs[i + 1][:2])  
        v1 = p_atual - p_anterior  # Vetor entre pontos
        v2 = p_atual - p_proximo    # Vetor entre pontos
        normal1 = np.array([-v1[1], v1[0]])  # Perpendicular ao vetor v1
        normal2 = -np.array([-v2[1], v2[0]])  # Perpendicular ao vetor v2
        if np.linalg.norm(normal1) != 0 and np.linalg.norm(normal2) != 0:  # Evita divisões por zero
            normal1 /= np.linalg.norm(normal1)  # Normaliza a normal
            normal2 /= np.linalg.norm(normal2)  # Normaliza a normal
            normalfinal = normal1 + normal2
            normalfinal /= np.linalg.norm(normalfinal)  # Normaliza a normal
        ##se a normal tiver direçao positiva para centro de massa inverter normal, senao nao afzer nada
        flagDirecaoNormal=verifica_direcao(normalfinal, p_atual, centroMAssa[:2])
        if flagDirecaoNormal== True:
            #aponta para centro de massa logo inverter
            normalAUtilizar=-normalfinal
        else:
            normalAUtilizar=normalfinal
        normais.append(normalAUtilizar)
        r,p,y=normal_to_rpy(normalAUtilizar)
        rpy.append([r,p,y])
    return np.array(normais), np.array(rpy)
#############################################################################################
def afastar_pontos(pontos_corrigidos_zs, d):
    pontos_xy = np.array(pontos_corrigidos_zs)[:, :2]  # penas X e Y
    
    cxCM, cyCM, czCM = np.mean(pontos_corrigidos_zs, axis=0) # Calcula a média ao longo da dimensão 0 o que resulta no centor de massa para x,y,z


    centroMAssa=(cxCM, cyCM,czCM) # Armazena o centro de massa nas variáveis
    print(f"Centro de massa (3D): ({centroMAssa})")# Imprime a posição do centro no espaço 3D
    normais,rpy = calcular_normais(pontos_corrigidos_zs ,centroMAssa) 
    # Afasta os pontos na direção das normais
    novos_pontos_xy = pontos_xy + (d * normais)
    novos_pontos = np.hstack([novos_pontos_xy, np.array(pontos_corrigidos_zs)[:, 2].reshape(-1, 1)])  # Mantém Z
    return novos_pontos, normais,rpy
#############################################################################################
def pontoMedioReta(pt1, pt2,image_rearengedT):
    # Separar posição e orientação
    pos1, quat1 = np.array(pt1[:3]), np.array(pt1[3:])
    pos2, quat2 = np.array(pt2[:3]), np.array(pt2[3:])

    # Média dos quaternions (simples soma e normalização) #nao é a forma mais correta de se fazer
    quat_media_raw = (quat1 + quat2) / 2.0
    norm = np.linalg.norm(quat_media_raw)
    quat_media = quat_media_raw / norm if norm > 0 else quat_media_raw

    pos11=pontoRealXYZ_para_pixel_referencial_P1(pos1[0],pos1[1],pos1[2])
    pos22=pontoRealXYZ_para_pixel_referencial_P1(pos2[0],pos2[1],pos2[2])
    xpix=round((pos11[0]+pos22[0])/2)
    ypix=round((pos11[1]+pos22[1])/2)

    media_pos=pixel_para_3D(xpix, ypix,image_rearengedT,intrinsics)
    media_pos=converter_ponto_para_referencial_origemMundo(media_pos)
   
    pontoMedio= np.array([])
    pontoMedio=np.hstack([media_pos,quat_media])
    return pontoMedio
 

def ponto_distante_na_reta(P1, P2, dist):
    P1 = np.array(P1)
    P2 = np.array(P2)
    # direção
    d = P2 - P1
    norm = np.linalg.norm(d)
    d_unit = d / norm
  
    # Novo ponto a distância 'a' a partir do ponto médio
    p1x=pontoRealXYZ_para_pixel_referencial_P1(P1[0],P1[1],P1[2])
    p2x=pontoRealXYZ_para_pixel_referencial_P1(P2[0],P2[1],P2[2])
    pmedx=(np.array(p1x)+np.array(p2x))/2
    difx=np.array(p1x)- np.array(p2x)
    difx=np.linalg.norm(difx)
    dif=np.array(P1[:2])-np.array(P2[:2])
    dif=np.linalg.norm(dif)
    relacao=dist/dif
    distx=difx*relacao
    d_unit=np.linalg.norm(d_unit[:2])

    # direção
    dx = np.array(p2x) - np.array(p1x)
    normx = np.linalg.norm(dx)

    d_unitx = dx / normx
    pax=pmedx+ distx*d_unitx
    x=round(pax[0])
    y=round(pax[1])

    Pa=pixel_para_3D(x, y,image_rearengedT,intrinsics)
    Pa=converter_ponto_para_referencial_origemMundo(Pa)
    Pa=np.array(Pa)
    return Pa





























"""Pega no contorno obtido da porta e projeta na torre. Fica com valores de x y e x no array pontos_3D """

#################################################################################################################################################################################
###############################################################################################################################################################################
###############################################################################################################################################################################
###############################################################################################################################################################################
# Transformando contorno detectado em coordenadas 3D
pontos_3D = np.zeros((1, 3)) # Inicializa uma lista vazia para armazenar pontos 3D
n=0
m=0
for ponto in maior_contorno_bag: # Precorre todos os pontos do contorno reduzido
    x, y = ponto[0], ponto[1]  # Obtém as coordenadas x e y de cada ponto do contorno
    ponto_3D = pixel_para_3D(x, y, image_rearengedT,intrinsics) # Converte as coordenadas do ponto de pixel para 3D usando a função 'pixel_para_3D'
    if ponto_3D: # Se a conversão foi bem sucedida o ponto 3D é adicionado à lista 'pontos_3D'
        if m==0:
            pontos_3D = np.array(ponto_3D).reshape(1, 3) 
            m+=1
        else:
            pontos_3D = np.vstack((pontos_3D,ponto_3D))   
    else:
        print("erro ponto 3D")
###############################################################################################################################################################################
###############################################################################################################################################################################
###############################################################################################################################################################################
###############################################################################################################################################################################
###############################################################################################################################################################################
###############################################################################################################################################################################
""" REDUÇAO DOS PONTOS POR APROXIMAÇAO LINEAR 3D """
#Inicialização das variáveis
i=0
dist_NaoAceite=False # Flag para indicar se a distância de um ponto à reta não é aceitável
a = 0 # Índice do primeiro ponto a ser considerado no array
l = 2 # Número de pontos a serem considerados na reta (começa com 2 e será aumentado conforme necessário)
b = 0 # Novo ponto criado
k=0   # Variável de controle para iterar sobre os pontos
dist_max = 0.4  # Distância máxima aceitável de cada ponto à reta
# Arrays de dados
arrayPontos = np.empty((0, 3)) # Array de pontos, inicialmente vazio
arrayPontosContornoFinal=np.empty((0, 3)) # Lista que armazenará o contorno final dos pontos após processamento
arrayZZsPontos=np.empty((0, 3))# Lista para armazenar pontos com base na distância ou algum critério adicional
pontoInicialTemp=0 # Ponto inicial temporário para referência durante o processo
pontoFinalTemp=0 # Ponto final temporário para referência
maior_contorno_bag=np.vstack((maior_contorno_bag , maior_contorno_bag[0,:]))

#############################################################
#############################################################
#############################################################
# Criando figura e gráfico 3D
if enablePlot==True:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1.0, 1.0, 1.0])
    scatter = ax.scatter(pontos_3D[:,0], pontos_3D[:,1], pontos_3D[:,2])
    set_axes_equal(ax)  
    plt.show()
#############################################################
#############################################################
#############################################################
#############################################################
# Inicia um loop que percorre todos os pontos em 'matrizpontosbag'
while i < len(pontos_3D):# and fim==True           :# Enquanto o índice i for menor que o tamanho da matriz de pontos
# Caso o índice i esteja dentro do intervalo de pontos que serão considerados para formar uma reta                           
        if i <= a+l-1: # Se o índice i ainda estiver dentro do intervalo de pontos a serem considerados para a reta
        	       # Adiciona o ponto atual ao array temporário de pontos
            if len(arrayPontos)==0:
                arrayPontos= pontos_3D[i, :3]
            else:
                arrayPontos = np.vstack((arrayPontos, pontos_3D[i, :3]))
            
            if i < a+l-1: # Se não for o último ponto a ser adicionado
                i+=1 # Incrementa o índice para o próximo ponto
   	# Caso tenha atingido o número de pontos desejados para formar a reta
        if i == a+l-1: # Quando tiver sido considerado o número de pontos definidos por 'a' e 'l' 
            # Cria uma reta usando os pontos contidos no arrayPontos com a função cv2.fitLine   
            if len(arrayPontos)==3:
                arrayPontos= np.vstack((arrayPontos, arrayPontos))
            reta1=cv2.fitLine(arrayPontos, cv2.DIST_L2, 0, 0.01, 0.01)
	    # Verifica a distância de cada ponto da reta com a função 'distancia_ponto_reta'
            distan =distancia_ponto_reta(arrayPontos , reta1 , dist_max) 
	    # Caso a distância seja aceitável (distancia_ponto_reta retorna False)
            if distan == False:
                xa,ya,za= arrayPontos[0] # Obtém as coordenadas do primeiro ponto
                xb,yb,zb=arrayPontos[len(arrayPontos)-1] # Obtém as coordenadas do último ponto
                xa=int(xa)
                xb=int(xb)
                ya=int(ya)
                yb=int(yb)
                za=int(za)
                zb=int(zb)
	    # Caso a distância não seja aceitável (distancia_ponto_reta retorna True)
            if distan ==True:
	    # Adiciona os pontos ao contorno final e as distâncias à lista de profundidades
                arrayPontosContornoFinal=np.vstack((arrayPontosContornoFinal, pontoInicialTemp))# Adiciona ponto inicial ao contorno final
                arrayPontosContornoFinal=np.vstack((arrayPontosContornoFinal, pontoFinalTemp))# Adiciona ponto final ao contorno final
		# Reinicia as variáveis de controle para considerar o próximo conjunto de pontos
                j=0 # Inicializa a variável a zero
                a=a+l # Atualiza o índice inicial para a próxima sequência de pontos
                l=2 # Define o número de pontos a considerar para a próxima reta
                i=a # Atualiza o índice i para começar do próximo ponto
                arrayPontos = np.empty((0, 3)) # Faz reset ao array de pontos temporário
                reta=0 # Faz reset à variável reta
                distan=0 # Faz reset à variável de distância
	    # Caso a reta formada não seja válida (distancia_ponto_reta retornou True)
            else:             
                i=a # Atualiza o índice 'i' para o próximo conjunto de pontos
                l+=1 # Incrementa o número de pontos a serem considerados para a reta
                #print("Ponto xxxxxxx=", arrayPontos[0])
 		# Calcula o novo primeiro ponto da reta usando a função ponto_mais_proximo
    		# A função ponto_mais_proximo recebe o ponto inicial do array de pontos e a reta
                pontoInicialTemp=ponto_mais_proximo_3D(arrayPontos[0,:], reta1)
		# Calcula o novo último ponto da reta
                pontoFinalTemp=ponto_mais_proximo_3D(arrayPontos[len(arrayPontos)-1,:], reta1)
		# Faz reset ao arrayPontos para começar o próximo conjunto de pontos
                arrayPontos=np.array([])
#print("Array definitivo concluido:", arrayPontosContornoFinal)
# Calcula o tamanho do array definitivo do contorno
tamanhoArray1=len(pontos_3D)
tamanhoArray2=len(arrayPontosContornoFinal)
# Mostra o número de pontos no contorno total (contorno original)
print(f"Número de pontos no contorno original: {tamanhoArray1}")
# Mostra o número de pontos no contorno após o processo de redução
print(f"Número de pontos no contorno reduzido: {tamanhoArray2}")
#############################################################
#############################################################
#############################################################
#############################################################
#Plot para visualizacao
if enablePlot==True:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1.0, 1.0, 1.0])
    scatter = ax.scatter(arrayPontosContornoFinal[:,0], arrayPontosContornoFinal[:,1], arrayPontosContornoFinal[:,2])
    set_axes_equal(ax)
    plt.show()
#############################################################
#############################################################
#############################################################
#############################################################
#####################Algoritmo afastar d cm os pontos########
#############################################################
################################################### 
#Matriz correspondente à correção do centro da camara face à camara de infravermelhos esquerda(?)
matriz_infravermelhoDireita2camara=np.eye(4)
matriz_infravermelhoDireita2camara[0,3]=0
matriz_infravermelhoDireita2camara[1,3]=0
matriz_infravermelhoDireita2camara[2,3]=0
 # Matriz correspondente à correção do centro da camara face ao tool0/punho robo


matriz_camara2flange = np.array( [
 [ 9.999e-01,-3.090e-03,-1.438e-02,-1.885e+01],
 [-1.438e-02,-2.426e-03,-9.999e-01,-1.087e+01],
 [ 3.055e-03, 1.000e+00,-2.470e-03, 2.806e+02],
 [ 0.000e+00, 0.000e+00, 0.000e+00, 1.000e+00]])
matriz_camara2flange_rot=matriz_camara2flange[:3,:3]
#Transformar quaternions de p2 em matriz de rotação homogenea
"""Confirmar esta matriz-->  CONST robtarget p1:=[[30.38,714.83,1454.47],[0.00014,-0.000446,-0.707356,-0.706858],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
"""
#robotastudio dá wxyz
#p1
quat_array =[ -0.000446,-0.707356,-0.706858, 0.00014] 
rot =   Rotation.from_quat(quat_array) 
matriz_p1_2_origemmundo_base = rot.as_matrix() 
matriz_p1_2_origemmundo=np.eye(4)
matriz_p1_2_origemmundo[0,3]=30.38
matriz_p1_2_origemmundo[1,3]=714.83
matriz_p1_2_origemmundo[2,3]=1454.47
matriz_p1_2_origemmundo[3,3]=1
matriz_p1_2_origemmundo[:3, :3] = matriz_p1_2_origemmundo_base

pontos_sem_offset_origem_corrigida = np.empty_like(arrayPontosContornoFinal)



print("A corrigir referencial de origem..")  
t=0


for point in tqdm(arrayPontosContornoFinal): 
    #Matriz correspondente aos targets sem correções- pontos_sem_offset
    matrizponto=np.array([  [point[0]],
                            [point[1]],
                            [point[2]],
                            [1]])

    matrizponto_corrigida_origem= matriz_p1_2_origemmundo @ matriz_camara2flange @ matriz_infravermelhoDireita2camara @ matrizponto
   
    ponto=np.array([matrizponto_corrigida_origem[0,0],
                    matrizponto_corrigida_origem[1,0],
                    matrizponto_corrigida_origem[2,0] ])
   
    pontos_sem_offset_origem_corrigida[t]= ponto
    t+=1


if enablePlot==True:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1.0, 1.0, 1.0])
    scatter = ax.scatter(pontos_sem_offset_origem_corrigida[:,0], pontos_sem_offset_origem_corrigida[:,1], pontos_sem_offset_origem_corrigida[:,2])
    set_axes_equal(ax)
    plt.show()



# Distância de afastamento (1 cm). Corte a direito
#distancia1 = 0.01  # 1 cm em metros
distancia1 = 10  #10mm =1cm

# Aplicar o afastamento
pontos_1st_offset_0graus, normais,rpy = afastar_pontos(pontos_sem_offset_origem_corrigida, distancia1)
print("Caminho zero graus concluido")

if enablePlot==True:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1.0, 1.0, 1.0])
    scatter = ax.scatter(pontos_1st_offset_0graus[:,0], pontos_1st_offset_0graus[:,1], pontos_1st_offset_0graus[:,2])   
    set_axes_equal(ax)
    plt.show()



 
qw=[]
qx=[]
qy=[]
qz=[]
for i in range(len(rpy)):
    roll = rpy[i,0]
    pitch = rpy[i,1]
    yaw = rpy[i,2]
 
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
 
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
 
    qw.append(w)
    qx.append(x)
    qy.append(y)
    qz.append(z)


q_cam = np.stack([qx, qy, qz, qw], axis=1)  # shape: (N, 4)
# Converter para matriz de rotação
R_target_cam = Rotation.from_quat(q_cam).as_matrix()  # shape: (N, 3, 3)
# Matriz de transformação global (flange->mundo @ cam->flange)
R_total_rot = matriz_p1_2_origemmundo[:3,:3] @ matriz_camara2flange_rot[:3,:3]  # shape: (3, 3)
# Aplicar a todos os R_target_cam — usar .T para garantir ordem correta
R_target_world = np.matmul(R_target_cam, R_total_rot.T)  # shape: (N, 3, 3)
# Converter para quaternions
q_world = Rotation.from_matrix(R_target_world).as_quat()  # shape: (N, 4)
# Separar os componentes
x = q_world[:, 0]
y = q_world[:, 1]
z = q_world[:, 2]
w = q_world[:, 3]

escala: float = 1
"""  target medido"""
for point in tqdm(pontos_sem_offset_origem_corrigida):   
   
    contorno_escalado0 = np.array([[point[0] * escala, point[1] * escala, point[2] * escala] for point in pontos_sem_offset_origem_corrigida])
qw=np.array(qw)
qx=np.array(qx)
qy=np.array(qy)
qz=np.array(qz)
tam=int( len(qw))
qw = qw.reshape(tam,-1)
qx = qx.reshape(tam,-1) 
qy = qy.reshape(tam,-1)
qz = qz.reshape(tam,-1) 
contorno_escalado0 = np.hstack((contorno_escalado0, qw, qx, qy, qz))

#################################################################
################################################################
#################################################################
################################################################
#         Dividir caminho em dois
#################################################################
################################################################
#################################################################
################################################################
xCM = np.mean(contorno_escalado0[:,0], axis=0) # Calcula a média dos x
yCM = np.mean(contorno_escalado0[:,1], axis=0) # Calcula a média dos y
centroMassa=np.array([[1,2]])
centroMassa[0,0] = xCM
centroMassa[0,1] = yCM

distAoCentroReta =80
margemDistanciaCentro=distAoCentroReta+30
###########################################################################################
filtro1 = (contorno_escalado0[:, 0] > xCM+margemDistanciaCentro) & (contorno_escalado0[:, 1] > yCM)
pontos_filtrados1 = contorno_escalado0[filtro1]
# Calcular distâncias ao centro de massa
distancias1 = np.linalg.norm(np.abs(pontos_filtrados1[:,:2] - centroMassa), axis=1)
# Encontrar o índice do ponto mais próximo
indice_B = np.argmin(distancias1)
pontoB= pontos_filtrados1[indice_B]
# Localizar esse ponto no array original (comparando x e y)
indice_B_contornoescalado0 = np.where(
    np.all(contorno_escalado0[:, :2] == pontoB[:2], axis=1)
)[0][0]
###########################################################################################
filtro2 = (contorno_escalado0[:, 0] < xCM-margemDistanciaCentro) & (contorno_escalado0[:, 1] < yCM)
pontos_filtrados2 = contorno_escalado0[filtro2]
# Calcular distâncias ao centro de massa
distancias2 = np.linalg.norm(np.abs(pontos_filtrados2[:,:2] - centroMassa), axis=1)
# Encontrar o índice do ponto mais próximo
indice_C = np.argmin(distancias2)
pontoC = pontos_filtrados2[indice_C]
# Localizar esse ponto no array original (comparando x e y)
indice_C_contornoescalado0 = np.where(
    np.all(contorno_escalado0[:, :2] == pontoC[:2], axis=1)
)[0][0]
###########################################################################################
filtro3 = (contorno_escalado0[:, 0] > xCM+margemDistanciaCentro) & (contorno_escalado0[:, 1] < yCM)
pontos_filtrados3 = contorno_escalado0[filtro3]
# Calcular distâncias ao centro de massa
distancias3 = np.linalg.norm(np.abs(pontos_filtrados3[:,:2] - centroMassa), axis=1)
# Encontrar o índice do ponto mais próximo
indice_A = np.argmin(distancias3)
pontoA = pontos_filtrados3[indice_A]
# Localizar esse ponto no array original (comparando x e y)
indice_A_contornoescalado0 = np.where(
    np.all(contorno_escalado0[:, :2] == pontoA[:2], axis=1)
)[0][0]
###########################################################################################
filtro4 = (contorno_escalado0[:, 0] < xCM-margemDistanciaCentro) & (contorno_escalado0[:, 1] > yCM)
pontos_filtrados4 = contorno_escalado0[filtro4]
# Calcular distâncias ao centro de massa
distancias4 = np.linalg.norm(np.abs(pontos_filtrados4[:,:2] - centroMassa), axis=1)
# Encontrar o índice do ponto mais próximo
indice_D = np.argmin(distancias4)
pontoD = pontos_filtrados4[indice_D]
# Localizar esse ponto no array original (comparando x e y)
indice_D_contornoescalado0 = np.where(
    np.all(contorno_escalado0[:, :2] == pontoD[:2], axis=1)
)[0][0]
###########################################################################################
"""CaminhoBA"""
if indice_B_contornoescalado0 > indice_A_contornoescalado0:
    caminhoBA = contorno_escalado0[indice_B_contornoescalado0:indice_A_contornoescalado0:-1]
    # Adiciona o ponto A no fim (porque slicing para trás exclui o stop)
    caminhoBA = np.vstack((caminhoBA, contorno_escalado0[indice_A_contornoescalado0]))
else:
 # De B até início (inclusive 0)
    parte1 = contorno_escalado0[indice_B_contornoescalado0::-1]
    # Do fim até A (inclusiveA)
    parte2 = contorno_escalado0[:indice_A_contornoescalado0-1:-1]
    # Juntar tudo
    caminhoBA = np.vstack((parte1, parte2, contorno_escalado0[indice_A_contornoescalado0]))

"""CaminhoCD"""
if indice_C_contornoescalado0 > indice_D_contornoescalado0:
    caminhoCD = contorno_escalado0[indice_C_contornoescalado0:indice_D_contornoescalado0:-1]
    caminhoCD = np.vstack((caminhoCD, contorno_escalado0[indice_D_contornoescalado0]))
else:
 # De C até início (inclusive 0)
    parte1 = contorno_escalado0[indice_C_contornoescalado0::-1]
    # Do fim até D (inclusive D)
    parte2 = contorno_escalado0[:indice_D_contornoescalado0-1:-1]
    caminhoCD = np.vstack((parte1, parte2, contorno_escalado0[indice_D_contornoescalado0]))





"""
Com ponto A e ponto C  :
Encontrar reta
Encontrar centro de reta
Offset para direção A em 150mm  juntar fim do caminho BA
Offset em direção normal interior 50mm juntar ao fim fim do caminho BA

Offset para direção C em 150mm  juntar inicio do caminho CD
Offset em direção normal interior 50mm juntar ao inicio do caminho CD



Com ponto B e ponto D  :
Encontrar reta
Encontrar centro de reta
Offset para direção D em 150mm  juntar fim do caminho CD 
Offset em direção normal interior 50mm juntar ao fim do caminho CD


Offset para direção B em 150mm  juntar inicio do caminho BA
Offset em direção normal interior 50mm juntar ao inicio do caminho BA

Temos 2 novos caminhos, apolicar as transformações aos 2 e corrigir rapid

"""

x,y=pontoRealXYZ_para_pixel_referencial_P1(pontoA[0],pontoA[1],pontoA[2])





pontoMedioBD = pontoMedioReta(pontoB, pontoD,image_rearengedT)
pontoMedioAC = pontoMedioReta(pontoA, pontoC,image_rearengedT)


pontoBDD = ponto_distante_na_reta(pontoB, pontoD, distAoCentroReta)
pontoBBD = ponto_distante_na_reta(pontoD, pontoB, distAoCentroReta)
pontoACC  = ponto_distante_na_reta(pontoA, pontoC, distAoCentroReta)
pontoAAC = ponto_distante_na_reta(pontoC, pontoA, distAoCentroReta)


pontoBDD= np.hstack((pontoBDD,pontoMedioBD[3:7] ))
pontoBBD= np.hstack((pontoBBD,pontoMedioBD[3:7]))
pontoAAC= np.hstack((pontoAAC,pontoMedioAC[3:7]))
pontoACC= np.hstack((pontoACC,pontoMedioAC[3:7]))


caminhoBA=np.vstack([ pontoBBD , caminhoBA, pontoAAC ])
caminhoCD=np.vstack([ pontoACC , caminhoCD, pontoBDD, ])
###############################################
###############################################
###############################################
###############################################
###############################################
#circunferencia corte curvo

distanciaL=50
#ponto A

pontoA_J=caminhoBA[-2]
pontoA_K=caminhoBA[-1]
pontoB_J=caminhoBA[1]
pontoB_K=caminhoBA[0]
pontoC_J=caminhoCD[1]
pontoC_K=caminhoCD[0]
pontoD_J=caminhoCD[-2]
pontoD_K=caminhoCD[-1]



#normal para interior.

def calculaNormalInteriorAfastamentoPonto(pontoJ,pontoK,centroMassa,distanciaL):
    vetor = pontoJ - pontoK  # Vetor entre pontos
    normal1 = np.array([-vetor[1], vetor[0]])  # Perpendicular ao vetor v1
    if np.linalg.norm(normal1) != 0 :  # Evita divisões por zero
        normal1 /= np.linalg.norm(normal1)  # Normaliza a normal
        normal1=normal1.reshape(2,)
        centroMassa=centroMassa.reshape(2,)
        ##se a normal tiver direçao negativa para centro de massa inverter normal, senao nao afzer nada
        flagDirecaoNormal=verifica_direcao(normal1, pontoK, centroMassa)
        if flagDirecaoNormal== True:
            #aponta para centro de massa logo inverter
            normalAUtilizar1=normal1
        else:
            normalAUtilizar1=-normal1
        pontoL=pontoK+normalAUtilizar1*distanciaL
        return pontoL
    else: print("ErroCalculoNormalInterior5cmcircunferencia")

pontoA_L=calculaNormalInteriorAfastamentoPonto(pontoA_J[:2],pontoA_K[:2],centroMassa,distanciaL)

pontoB_L=calculaNormalInteriorAfastamentoPonto(pontoB_J[:2],pontoB_K[:2],centroMassa,distanciaL)
pontoC_L=calculaNormalInteriorAfastamentoPonto(pontoC_J[:2],pontoC_K[:2],centroMassa,distanciaL)
pontoD_L=calculaNormalInteriorAfastamentoPonto(pontoD_J[:2],pontoD_K[:2],centroMassa,distanciaL)

pontoA_N=calculaNormalInteriorAfastamentoPonto(pontoA_K[:2],pontoA_L[:2],caminhoCD[0,:2],distanciaL)
pontoB_N=calculaNormalInteriorAfastamentoPonto(pontoB_K[:2],pontoB_L[:2],caminhoCD[-1,:2],distanciaL)
pontoC_N=calculaNormalInteriorAfastamentoPonto(pontoC_K[:2],pontoC_L[:2],caminhoBA[-1,:2],distanciaL)
pontoD_N=calculaNormalInteriorAfastamentoPonto(pontoD_K[:2],pontoD_L[:2],caminhoBA[0,:2],distanciaL)




pontoANfinal=np.zeros_like(pontoAAC)
pontoANfinal[0:2]=pontoA_N[0:2]
pontoANfinal[2:7]=pontoAAC[2:7]

pontoBNfinal=np.zeros_like(pontoBBD)
pontoBNfinal[0:2]=pontoB_N[0:2]
pontoBNfinal[2:7]=pontoBBD[2:7]

pontoCNfinal=np.zeros_like(pontoACC)
pontoCNfinal[0:2]=pontoC_N[0:2]
pontoCNfinal[2:7]=pontoACC[2:7]

pontoDNfinal=np.zeros_like(pontoBDD)
pontoDNfinal[0:2]=pontoD_N[0:2]
pontoDNfinal[2:7]=pontoBDD[2:7]



def criarPonto45(pontoK, pontoL, pontoN, distanciaL):

    vLK = pontoK[:2] - pontoL[:2]  # Vetor entre pontos
    vLN = pontoN[:2] - pontoL[:2]   # Vetor entre pontos
    normalLK = np.array([-vLK[1], vLK[0]])  # Perpendicular ao vetor v1
    normalLN = -np.array([-vLN[1], vLN[0]])  # Perpendicular ao vetor v2
    if np.linalg.norm(normalLK) != 0 and np.linalg.norm(normalLN) != 0:  # Evita divisões por zero

        normalLK /= np.linalg.norm(normalLK)  # Normaliza a normal
        normalLN /= np.linalg.norm(normalLN)  # Normaliza a normal
        normalLM = normalLK + normalLN
        normalLM /= np.linalg.norm(normalLM)  # Normaliza a normal
             ##se a normal tiver direçao negativa para centro de massa inverter normal, senao nao afzer nada
        normalLM=normalLM.reshape(2,)
        flagDirecaoNormal=verifica_direcao(normalLM, pontoL, pontoK[:2]) 

        if flagDirecaoNormal== True:
            #aponta para centro de massa logo inverter
            normalAUtilizar2=normalLM
        else:
            normalAUtilizar2=-normalLM

    pontoM=pontoL + normalAUtilizar2 * distanciaL   
    return pontoM
#ponto na normal para interior 5cm
#normal da normal para interior
#ponto da normal da normal para interior 5 cm



def criarretas(pontoinicial, pontofinal,pontomedio):
    distUnit=10
    n=1
    a=0
   
    p_Ini_reta=pontoRealXYZ_para_pixel_referencial_P1(pontoinicial[0],pontoinicial[1], pontoinicial[2])
    p_Fin_reta=pontoRealXYZ_para_pixel_referencial_P1(pontofinal[0],pontofinal[1], pontofinal[2])
        
    # direção
    dx = np.array(p_Fin_reta) - np.array(p_Ini_reta)
    normx = np.linalg.norm(dx)
    d_unitx = dx / normx
    pontosreta=np.array([p_Ini_reta[:2]])
    
    while a==0:
        p_novo=p_Ini_reta[:2]+ n*distUnit*d_unitx

        n+=1
        if n*distUnit<normx-1:
            pontosreta=np.vstack((pontosreta,p_novo))
        else:
            pontosreta=np.vstack((pontosreta,p_Fin_reta[:2]))
            pontosretafinal=np.array(())
            b=0
            a=1  
    for ponto in pontosreta:
        xx=int(round(ponto[0]))
        yy=int(round(ponto[1]))
        ponto=np.hstack((ponto,0))
        ponto[0:3]=pixel_para_3D(xx, yy,image_rearengedT,intrinsics)
        ponto=converter_ponto_para_referencial_origemMundo(ponto[0:3])
        ponto=np.array(ponto)
                
        ponto=[ponto[0],ponto[1],ponto[2],0,0,0,0]
        ponto[3:7]=pontomedio[3:7]
        if b==0:
            pontosretafinal= ponto  
            b+=1
        else:
            pontosretafinal=np.vstack((pontosretafinal, ponto))
            
    return pontosretafinal

###########################################################################

pontoAM= criarPonto45(pontoA_K, pontoA_L, pontoANfinal, distanciaL)
pontoBM= criarPonto45(pontoB_K, pontoB_L, pontoBNfinal, distanciaL)
pontoCM= criarPonto45(pontoC_K, pontoC_L, pontoCNfinal, distanciaL)
pontoDM= criarPonto45(pontoD_K, pontoD_L, pontoDNfinal, distanciaL)
 
pontoAMFinal=np.zeros_like(pontoA)
pontoAMFinal[0:2]=pontoAM[0:2]
pontoAMFinal[2:7]=pontoAAC[2:7]

pontoBMFinal=np.zeros_like(pontoB)
pontoBMFinal[0:2]=pontoBM[0:2]
pontoBMFinal[2:7]=pontoBBD[2:7]

pontoCMFinal=np.zeros_like(pontoC)
pontoCMFinal[0:2]=pontoCM[0:2]
pontoCMFinal[2:7]=pontoACC[2:7]

pontoDMFinal=np.zeros_like(pontoD)
pontoDMFinal[0:2]=pontoDM[0:2]
pontoDMFinal[2:7]=pontoBDD[2:7]

pontos_entre_AJ_e_AK=criarretas(pontoA_J,pontoA_K,pontoA_K) 
if np.all(pontos_entre_AJ_e_AK[-1,:2]==caminhoBA[0,:2]):
    pontos_entre_AJ_e_AK= pontos_entre_AJ_e_AK[1:-1]
    pontos_entre_AJ_e_AK= pontos_entre_AJ_e_AK[::-1]
if np.all(pontos_entre_AJ_e_AK[-1,:2]==caminhoBA[-1,:2]):
    pontos_entre_AJ_e_AK= pontos_entre_AJ_e_AK[1:-1]


pontos_entre_BJ_e_BK=criarretas(pontoB_J,pontoB_K,pontoB_K)
if np.all(pontos_entre_BJ_e_BK[-1,:2]==caminhoBA[0,:2]):
    pontos_entre_BJ_e_BK= pontos_entre_BJ_e_BK[1:-1]
    pontos_entre_BJ_e_BK= pontos_entre_BJ_e_BK[::-1]
if np.all(pontos_entre_BJ_e_BK[-1,:2]==caminhoBA[-1,:2]):
    pontos_entre_BJ_e_BK= pontos_entre_BJ_e_BK[1:-1]


pontos_entre_CJ_e_CK=criarretas(pontoC_J,pontoC_K,pontoC_K)  
if np.all(pontos_entre_CJ_e_CK[-1,:2]==caminhoCD[0,:2]):
    pontos_entre_CJ_e_CK= pontos_entre_CJ_e_CK[1:-1]
    pontos_entre_CJ_e_CK= pontos_entre_CJ_e_CK[::-1]
if np.all(pontos_entre_CJ_e_CK[-1,:2]==caminhoCD[-1,:2]):
    pontos_entre_CJ_e_CK= pontos_entre_CJ_e_CK[1:-1]


pontos_entre_DJ_e_DK=criarretas(pontoD_J,pontoD_K,pontoD_K)   
if np.all(pontos_entre_DJ_e_DK[-1,:2]==caminhoCD[0,:2]):
    pontos_entre_DJ_e_DK= pontos_entre_DJ_e_DK[1:-1]
    pontos_entre_DJ_e_DK= pontos_entre_DJ_e_DK[::-1]
if np.all(pontos_entre_DJ_e_DK[-1,:2]==caminhoCD[-1,:2]):
    pontos_entre_DJ_e_DK= pontos_entre_DJ_e_DK[1:-1]


if enablePlot==True:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1.0, 1.0, 1.0])
    scatter = ax.scatter(caminhoBA[:,0], caminhoBA[:,1], caminhoBA[:,2])
    set_axes_equal(ax)
    plt.show()

caminhoBA=np.vstack([ pontoBNfinal , pontoBMFinal ,pontos_entre_BJ_e_BK, caminhoBA[1:-1], pontos_entre_AJ_e_AK, pontoAMFinal , pontoANfinal ])
caminhoCD=np.vstack([ pontoCNfinal , pontoCMFinal ,pontos_entre_CJ_e_CK, caminhoCD[1:-1], pontos_entre_DJ_e_DK, pontoDMFinal , pontoDNfinal ])

###############################################
###############################################
###############################################
###############################################
"""caminhos e pontos para as retas"""
###############################################
###############################################
pontoA_L=calculaNormalInteriorAfastamentoPonto(pontoA_J[:2],pontoA_K[:2],centroMassa,distanciaL)
pontoB_L=calculaNormalInteriorAfastamentoPonto(pontoB_J[:2],pontoB_K[:2],centroMassa,distanciaL)
pontoC_L=calculaNormalInteriorAfastamentoPonto(pontoC_J[:2],pontoC_K[:2],centroMassa,distanciaL)
pontoD_L=calculaNormalInteriorAfastamentoPonto(pontoD_J[:2],pontoD_K[:2],centroMassa,distanciaL)

pontoA_N2=calculaNormalInteriorAfastamentoPonto(pontoA_K[:2],pontoA_L[:2],caminhoCD[0,:2],-distanciaL)
pontoB_N2=calculaNormalInteriorAfastamentoPonto(pontoB_K[:2],pontoB_L[:2],caminhoCD[-1,:2],-distanciaL)
pontoC_N2=calculaNormalInteriorAfastamentoPonto(pontoC_K[:2],pontoC_L[:2],caminhoBA[-1,:2],-distanciaL)
pontoD_N2=calculaNormalInteriorAfastamentoPonto(pontoD_K[:2],pontoD_L[:2],caminhoBA[0,:2],-distanciaL)

pontoAN2final=np.zeros_like(pontoAAC)
pontoAN2final[0:2]=pontoA_N2[0:2]
pontoAN2final[2:7]=pontoAAC[2:7]

pontoBN2final=np.zeros_like(pontoBBD)
pontoBN2final[0:2]=pontoB_N2[0:2]
pontoBN2final[2:7]=pontoBBD[2:7]

pontoCN2final=np.zeros_like(pontoACC)
pontoCN2final[0:2]=pontoC_N2[0:2]
pontoCN2final[2:7]=pontoACC[2:7]

pontoDN2final=np.zeros_like(pontoBDD)
pontoDN2final[0:2]=pontoD_N2[0:2]
pontoDN2final[2:7]=pontoBDD[2:7]




pontoAM2= criarPonto45(pontoA_K, pontoA_L, pontoAN2final, distanciaL)
pontoBM2= criarPonto45(pontoB_K, pontoB_L, pontoBN2final, distanciaL)
pontoCM2= criarPonto45(pontoC_K, pontoC_L, pontoCN2final, distanciaL)
pontoDM2= criarPonto45(pontoD_K, pontoD_L, pontoDN2final, distanciaL)
 
pontoAM2Final=np.zeros_like(pontoA)
pontoAM2Final[0:2]=pontoAM2[0:2]
pontoAM2Final[2:7]=pontoAAC[2:7]

pontoBM2Final=np.zeros_like(pontoB)
pontoBM2Final[0:2]=pontoBM2[0:2]
pontoBM2Final[2:7]=pontoBBD[2:7]

pontoCM2Final=np.zeros_like(pontoC)
pontoCM2Final[0:2]=pontoCM2[0:2]
pontoCM2Final[2:7]=pontoACC[2:7]

pontoDM2Final=np.zeros_like(pontoD)
pontoDM2Final[0:2]=pontoDM2[0:2]
pontoDM2Final[2:7]=pontoBDD[2:7]

#desde ks corrigir retas. reverter e ir pixel a pixel atraves de uma reta 

"""tentativa correçao reta AC"""

  
pontosretaAC=criarretas(pontoA_K,pontoC_K,pontoMedioAC)
pontosretaBD=criarretas(pontoB_K,pontoD_K,pontoMedioBD)
 

retaAC=np.vstack([pontoAN2final , pontoAM2Final , pontosretaAC , pontoCM2Final , pontoCN2final])
retaBD=np.vstack([pontoBN2final , pontoBM2Final , pontosretaBD , pontoDM2Final , pontoDN2final])






"""  targets RETAS"""

###############################################
###############################################
###############################################

"""  targets a 0 graus"""
for point in caminhoBA: 
   contorno_escalado0 = np.array([[point[0] * escala, point[1] * escala, point[2] * escala, point[3], point[4], point[5], point[6]] for point in caminhoBA])
for point in caminhoCD: 
   contorno_escalado1 = np.array([[point[0] * escala, point[1] * escala, point[2] * escala, point[3], point[4], point[5], point[6]] for point in caminhoCD])
for point in retaAC: 
   contorno_retaAC = np.array([[point[0] * escala, point[1] * escala, point[2] * escala, point[3], point[4], point[5], point[6]] for point in retaAC])
for point in retaBD: 
   contorno_retaBD = np.array([[point[0] * escala, point[1] * escala, point[2] * escala, point[3], point[4], point[5], point[6]] for point in retaBD])





offsetGeralZ=15

"""Offset pontos curcos dos inicios e fins dos contornos"""
for f in range(len(contorno_escalado0)):
    contorno_escalado0[f,2] = contorno_escalado0[f,2]+offsetGeralZ
   
for g in range(len(contorno_escalado1)):
        contorno_escalado1[g,2] = contorno_escalado1[g,2]+offsetGeralZ

for h in range(len(contorno_retaAC)):
    contorno_retaAC[h,2] = contorno_retaAC[h,2]+offsetGeralZ
   
for u in range(len( contorno_retaBD)):
         contorno_retaBD[u,2] =  contorno_retaBD[u,2]+offsetGeralZ


""""""

"""  targets a 30 graus"""
#Matrizes 30 graus   #avanço para baixo d1--> rotação a2--> recuo d3
#avanço para baixo d1
d11=20 #valor a confirmar! 
matriz_trans_d1=np.eye(4)
matriz_trans_d1[2,3]=d11
#rotação o2

o21=15  #graus
o21rad=np.deg2rad(o21)  #rad
matriz_rot_o2=np.eye(4)
matriz_rot_o2[0,0]=np.cos(o21rad)
matriz_rot_o2[0,2]=np.sin(o21rad)
matriz_rot_o2[2,0]=-np.sin(o21rad)
matriz_rot_o2[2,2]=np.cos(o21rad)
#recuo d3
d31=-(d11/np.cos(o21rad)) #valor a confirmar! 
matriz_trans_d3=np.eye(4)
matriz_trans_d3[2,3]=d31



pontos_30grausBA = np.zeros((len(contorno_escalado0), 7))
pontos_30grausCD = np.zeros((len(contorno_escalado1), 7))
pontos_30grausretaAC = np.zeros((len(contorno_retaAC), 7))
pontos_30grausretaBD = np.zeros((len(contorno_retaBD), 7))

for f in range(len(contorno_escalado1)):
    x, y, z, qx, qy, qz, qw = contorno_escalado1[f]

    # --- POSIÇÃO ---
    # Cria matriz homogénea original
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    T[:3, :3] = rot.from_quat([qx, qy, qz, qw]).as_matrix()

    # Translação d1 para baixo no eixo Z local
    T_local_d1 = np.eye(4)
    T_local_d1[2, 3] = d11

    # Rotação 30° no eixo Y local
    T_local_rot_y = np.eye(4)
    R_y = rot.from_euler('y', o21, degrees=True).as_matrix()
    T_local_rot_y[:3, :3] = R_y

    # Translação d3 no novo eixo Z local (depois da rotação)
    T_local_d3 = np.eye(4)
    T_local_d3[2, 3] = d31

    # Aplicar tudo no referencial local:
    T_novo = T @ T_local_d1 @ T_local_rot_y @ T_local_d3

    # Extrair nova posição e orientação
    nova_posicao = T_novo[:3, 3]
    nova_orientacao = rot.from_matrix(T_novo[:3, :3]).as_quat()

    # Guardar
    pontos_30grausCD[f] = [*nova_posicao, *nova_orientacao]


for f in range(len(contorno_escalado0)):
    x, y, z, qx, qy, qz, qw = contorno_escalado0[f]

    # --- POSIÇÃO ---
    # Cria matriz homogénea original
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    T[:3, :3] = rot.from_quat([qx, qy, qz, qw]).as_matrix()

    # Translação d1 para baixo no eixo Z local
    T_local_d1 = np.eye(4)
    T_local_d1[2, 3] = d11

    # Rotação 30° no eixo Y local
    T_local_rot_y = np.eye(4)
    R_y = rot.from_euler('y', o21, degrees=True).as_matrix()
    T_local_rot_y[:3, :3] = R_y

    # Translação d3 no novo eixo Z local (depois da rotação)
    T_local_d3 = np.eye(4)
    T_local_d3[2, 3] = d31

    # Aplicar tudo no referencial local:
    T_novo = T @ T_local_d1 @ T_local_rot_y @ T_local_d3

    # Extrair nova posição e orientação
    nova_posicao = T_novo[:3, 3]
    nova_orientacao = rot.from_matrix(T_novo[:3, :3]).as_quat()

    # Guardar
    pontos_30grausBA[f] = [*nova_posicao, *nova_orientacao]




for f in range(len(contorno_retaAC)):
    x, y, z, qx, qy, qz, qw = contorno_retaAC[f]

    # --- POSIÇÃO ---
    # Cria matriz homogénea original
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    T[:3, :3] = rot.from_quat([qx, qy, qz, qw]).as_matrix()

    # Translação d1 para baixo no eixo Z local
    T_local_d1 = np.eye(4)
    T_local_d1[2, 3] = d11

    # Rotação 30° no eixo Y local
    T_local_rot_y = np.eye(4)
    R_y = rot.from_euler('y', o21, degrees=True).as_matrix()
    T_local_rot_y[:3, :3] = R_y

    # Translação d3 no novo eixo Z local (depois da rotação)
    T_local_d3 = np.eye(4)
    T_local_d3[2, 3] = d31

    # Aplicar tudo no referencial local:
    T_novo = T @ T_local_d1 @ T_local_rot_y @ T_local_d3

    # Extrair nova posição e orientação
    nova_posicao = T_novo[:3, 3]
    nova_orientacao = rot.from_matrix(T_novo[:3, :3]).as_quat()

    # Guardar
    pontos_30grausretaAC[f] = [*nova_posicao, *nova_orientacao]

for f in range(len(contorno_retaBD)):
    x, y, z, qx, qy, qz, qw = contorno_retaBD[f]

    # --- POSIÇÃO ---
    # Cria matriz homogénea original
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    T[:3, :3] = rot.from_quat([qx, qy, qz, qw]).as_matrix()

    # Translação d1 para baixo no eixo Z local
    T_local_d1 = np.eye(4)
    T_local_d1[2, 3] = d11

    # Rotação 30° no eixo Y local
    T_local_rot_y = np.eye(4)
    R_y = rot.from_euler('y', o21, degrees=True).as_matrix()
    T_local_rot_y[:3, :3] = R_y

    # Translação d3 no novo eixo Z local (depois da rotação)
    T_local_d3 = np.eye(4)
    T_local_d3[2, 3] = d31

    # Aplicar tudo no referencial local:
    T_novo = T @ T_local_d1 @ T_local_rot_y @ T_local_d3

    # Extrair nova posição e orientação
    nova_posicao = T_novo[:3, 3]
    nova_orientacao = rot.from_matrix(T_novo[:3, :3]).as_quat()

    # Guardar
    pontos_30grausretaBD[f] = [*nova_posicao, *nova_orientacao]




"""  targets a menos 30 graus"""
#Matrizes 30 graus   #avanço para baixo d1--> rotação a2--> recuo d3
#avanço para baixo d1
d12=20 #valor a confirmar! 
matriz_trans_d1=np.eye(4)
matriz_trans_d1[2,3]=d12
#rotação o2
o22=-15 #graus
o22rad=np.deg2rad(o22)  #radianos
matriz_rot_o2=np.eye(4)
matriz_rot_o2[0,0]=np.cos(o22rad)
matriz_rot_o2[0,2]=np.sin(o22rad)
matriz_rot_o2[2,0]=-np.sin(o22rad)
matriz_rot_o2[2,2]=np.cos(o22rad)
#recuo d3
d32=-(d12/np.cos(o22rad))  #valor a confirmar! 
matriz_trans_d3=np.eye(4)
matriz_trans_d3[2,3]=d32








pontos_menos30grausBA = np.zeros((len(contorno_escalado0), 7))
pontos_menos30grausCD = np.zeros((len(contorno_escalado1), 7))
pontos_menos30grausretaAC = np.zeros((len(contorno_retaAC), 7))
pontos_menos30grausretaBD = np.zeros((len(contorno_retaBD), 7))

for f in range(len(contorno_escalado0)):
    x, y, z, qx, qy, qz, qw = contorno_escalado0[f]

    # --- POSIÇÃO ---
    # Cria matriz homogénea original
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    T[:3, :3] = rot.from_quat([qx, qy, qz, qw]).as_matrix()

    # Translação d1 para baixo no eixo Z local
    T_local_d1 = np.eye(4)
    T_local_d1[2, 3] = d12

    # Rotação 30° no eixo Y local
    T_local_rot_y = np.eye(4)
    R_y = rot.from_euler('y', o22, degrees=True).as_matrix()
    T_local_rot_y[:3, :3] = R_y

    # Translação d3 no novo eixo Z local (depois da rotação)
    T_local_d3 = np.eye(4)
    T_local_d3[2, 3] = d32

    # Aplicar tudo no referencial local:
    T_novo = T @ T_local_d1 @ T_local_rot_y @ T_local_d3

    # Extrair nova posição e orientação
    nova_posicao = T_novo[:3, 3]
    nova_orientacao = rot.from_matrix(T_novo[:3, :3]).as_quat()

    # Guardar
    pontos_menos30grausBA[f] = [*nova_posicao, *nova_orientacao]


for f in range(len(contorno_escalado1)):
    x, y, z, qx, qy, qz, qw = contorno_escalado1[f]

    # --- POSIÇÃO ---
    # Cria matriz homogénea original
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    T[:3, :3] = rot.from_quat([qx, qy, qz, qw]).as_matrix()

    # Translação d1 para baixo no eixo Z local
    T_local_d1 = np.eye(4)
    T_local_d1[2, 3] = d12

    # Rotação 30° no eixo Y local
    T_local_rot_y = np.eye(4)
    R_y = rot.from_euler('y', o22, degrees=True).as_matrix()
    T_local_rot_y[:3, :3] = R_y

    # Translação d3 no novo eixo Z local (depois da rotação)
    T_local_d3 = np.eye(4)
    T_local_d3[2, 3] = d32

    # Aplicar tudo no referencial local:
    T_novo = T @ T_local_d1 @ T_local_rot_y @ T_local_d3

    # Extrair nova posição e orientação
    nova_posicao = T_novo[:3, 3]
    nova_orientacao = rot.from_matrix(T_novo[:3, :3]).as_quat()

    # Guardar
    pontos_menos30grausCD[f] = [*nova_posicao, *nova_orientacao]






for f in range(len(contorno_retaAC)):
    x, y, z, qx, qy, qz, qw = contorno_retaAC[f]

    # --- POSIÇÃO ---
    # Cria matriz homogénea original
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    T[:3, :3] = rot.from_quat([qx, qy, qz, qw]).as_matrix()

    # Translação d1 para baixo no eixo Z local
    T_local_d1 = np.eye(4)
    T_local_d1[2, 3] = d12

    # Rotação 30° no eixo Y local
    T_local_rot_y = np.eye(4)
    R_y = rot.from_euler('y', o22, degrees=True).as_matrix()
    T_local_rot_y[:3, :3] = R_y

    # Translação d3 no novo eixo Z local (depois da rotação)
    T_local_d3 = np.eye(4)
    T_local_d3[2, 3] = d32

    # Aplicar tudo no referencial local:
    T_novo = T @ T_local_d1 @ T_local_rot_y @ T_local_d3

    # Extrair nova posição e orientação
    nova_posicao = T_novo[:3, 3]
    nova_orientacao = rot.from_matrix(T_novo[:3, :3]).as_quat()

    # Guardar
    pontos_menos30grausretaAC[f] = [*nova_posicao, *nova_orientacao]



for f in range(len(contorno_retaBD)):
    x, y, z, qx, qy, qz, qw = contorno_retaBD[f]

    # --- POSIÇÃO ---
    # Cria matriz homogénea original
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    T[:3, :3] = rot.from_quat([qx, qy, qz, qw]).as_matrix()

    # Translação d1 para baixo no eixo Z local
    T_local_d1 = np.eye(4)
    T_local_d1[2, 3] = d12

    # Rotação 30° no eixo Y local
    T_local_rot_y = np.eye(4)
    R_y = rot.from_euler('y', o22, degrees=True).as_matrix()
    T_local_rot_y[:3, :3] = R_y

    # Translação d3 no novo eixo Z local (depois da rotação)
    T_local_d3 = np.eye(4)
    T_local_d3[2, 3] = d32

    # Aplicar tudo no referencial local:
    T_novo = T @ T_local_d1 @ T_local_rot_y @ T_local_d3

    # Extrair nova posição e orientação
    nova_posicao = T_novo[:3, 3]
    nova_orientacao = rot.from_matrix(T_novo[:3, :3]).as_quat()

    # Guardar
    pontos_menos30grausretaBD[f] = [*nova_posicao, *nova_orientacao]











##################             #######################      ####################     #####    ############
####################           #######################      ####################     #####    #################
######      ##########         ########      #########      #######      #######     #####    ########  ##########
#####################          ########      #########      #######      #######     #####    ########      ########
###################            #######################      ####################     #####    ########       #######
########      ########         #######################      ####################     #####    ########       #######
#########      #########       #######################      #######                  #####    ########      ########
#########         #######      ########      #########      #######                  #####    ########  ###########
#########          ########    ########      #########      #######                  #####    #################
##########         #########   ########      #########      #######                  #####    ##############


#Código Rapid
print("A iniciar construção de código RAPID")


print("A construir RAPID...")  
##############################################

tool: str = "ClbTool"
wobj: str = "wobj0"
speed: int
zone: float
targetnumber0: int = 0
targetnumber1: int = 0
targetnumber2: int = 0
targetnumber3: int = 0
targetnumber4: int = 0
targetnumber5: int = 0
targetnumber6: int = 0
targetnumber7: int = 0
targetnumber8: int = 0
targetnumber9: int = 0
targetnumber10: int = 0
targetnumber11: int = 0
#z: float = 1712.22
#z: float = 1552.22
#criar ficheiro txt para depois meter no rapid
with open('GeneratedFiles_OCVCalculos_25/ficheiroRAPID', 'w') as ficheiro:
    ##definir os targets no module 1
    ficheiro.write("MODULE AWIND1 \n")
    ficheiro.write("    PERS tooldata ClbTool:=[TRUE,[[0,0,340],[1,0,0,0]],[1,[0,0,120],[1,0,0,0],0,0,0]]; \n")


    x=contorno_escalado0[0,0]
    y=contorno_escalado0[0,1]
    z=contorno_escalado0[0,2]+100
    qw=contorno_escalado0[0,6]
    qx=contorno_escalado0[0,3]
    qy=contorno_escalado0[0,4]
    qz=contorno_escalado0[0,5] 
    ficheiro.write(
        "    CONST robtarget a := [" +
        f"[{x:.4f},{y:.4f},{z:.4f}]," +
        f"[{qw:.7f},{qx:.7f},{qy:.7f},{qz:.7f}]," +
        "[-2,-1,1,2]," +
        "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
    ficheiro.write("    CONST robtarget p1:=[[30.38,714.83,1454.47],[0.00014,-0.000446,-0.707356,-0.706858],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
    #targets corte 0 graus
    for ponto in contorno_escalado0:
        x=ponto[0]
        y=ponto[1]
        z=ponto[2]
        qw=ponto[6]
        qx=ponto[3]
        qy=ponto[4]
        qz=ponto[5]        
        ficheiro.write(
            f"    CONST robtarget t0AB{targetnumber0} := [" +
            f"[{x:.4f},{y:.4f},{z:.4f}]," +
            f"[{qw:.7f},{qx:.7f},{qy:.7f},{qz:.7f}]," +
            "[-2,-1,1,2]," +
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
        targetnumber0+=1

    for ponto in contorno_escalado1:
        x=ponto[0]
        y=ponto[1]
        z=ponto[2]
        qw=ponto[6]
        qx=ponto[3]
        qy=ponto[4]
        qz=ponto[5]
        ficheiro.write(
            f"    CONST robtarget t0CD{targetnumber1} := [" +
            f"[{x:.4f},{y:.4f},{z:.4f}]," +
            f"[{qw:.7f},{qx:.7f},{qy:.7f},{qz:.7f}]," +
            "[-2,-1,1,2]," +
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
        targetnumber1+=1

#targets corte 30 graus
    for ponto in pontos_30grausBA:
        x=ponto[0]
        y=ponto[1]
        z=ponto[2]
        qw=ponto[6]
        qx=ponto[3]
        qy=ponto[4]
        qz=ponto[5]
        ficheiro.write(
            f"    CONST robtarget t30BA{targetnumber2} := [" +
            f"[{x:.4f},{y:.4f},{z:.4f}]," +
            f"[{qw:.7f},{qx:.7f},{qy:.7f},{qz:.7f}]," +
            "[-2,-1,1,2]," +
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
        targetnumber2+=1
    for ponto in pontos_30grausCD:
        x=ponto[0]
        y=ponto[1]
        z=ponto[2]
        qw=ponto[6]
        qx=ponto[3]
        qy=ponto[4]
        qz=ponto[5]
        ficheiro.write(
            f"    CONST robtarget t30CD{targetnumber3} := [" +
            f"[{x:.4f},{y:.4f},{z:.4f}]," +
            f"[{qw:.7f},{qx:.7f},{qy:.7f},{qz:.7f}]," +
            "[-2,-1,1,2]," +
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
        targetnumber3+=1        

#targets corte 30 graus
    for ponto in pontos_menos30grausBA:
        x=ponto[0]
        y=ponto[1]
        z=ponto[2]
        qw=ponto[6]
        qx=ponto[3]
        qy=ponto[4]
        qz=ponto[5]
        ficheiro.write(
            f"    CONST robtarget tm30BA{targetnumber4} := [" +
            f"[{x:.4f},{y:.4f},{z:.4f}]," +
            f"[{qw:.7f},{qx:.7f},{qy:.7f},{qz:.7f}]," +
            "[-2,-1,1,2]," +
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
        targetnumber4+=1

    for ponto in pontos_menos30grausCD:
        x=ponto[0]
        y=ponto[1]
        z=ponto[2]
        qw=ponto[6]
        qx=ponto[3]
        qy=ponto[4]
        qz=ponto[5]
        ficheiro.write(
            f"    CONST robtarget tm30CD{targetnumber5} := [" +
            f"[{x:.4f},{y:.4f},{z:.4f}]," +
            f"[{qw:.7f},{qx:.7f},{qy:.7f},{qz:.7f}]," +
            "[-2,-1,1,2]," +
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
        targetnumber5+=1

           #targets corte 0 graus
    for ponto in contorno_retaAC:
        x=ponto[0]
        y=ponto[1]
        z=ponto[2]
        qw=ponto[6]
        qx=ponto[3]
        qy=ponto[4]
        qz=ponto[5] 
        ficheiro.write(
            f"    CONST robtarget r0AC{targetnumber6} := [" +
            f"[{x:.4f},{y:.4f},{z:.4f}]," +
            f"[{qw:.7f},{qx:.7f},{qy:.7f},{qz:.7f}]," +
            "[-2,-1,1,2]," +
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
        targetnumber6+=1

    for ponto in contorno_retaBD:
        x=ponto[0]
        y=ponto[1]
        z=ponto[2]
        qw=ponto[6]
        qx=ponto[3]
        qy=ponto[4]
        qz=ponto[5]
        ficheiro.write(
            f"    CONST robtarget r0BD{targetnumber7} := [" +
            f"[{x:.4f},{y:.4f},{z:.4f}]," +
            f"[{qw:.7f},{qx:.7f},{qy:.7f},{qz:.7f}]," +
            "[-2,-1,1,2]," +
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
        targetnumber7+=1

    for ponto in pontos_30grausretaAC:
        x=ponto[0]
        y=ponto[1]
        z=ponto[2]
        qw=ponto[6]
        qx=ponto[3]
        qy=ponto[4]
        qz=ponto[5]   
        ficheiro.write(
            f"    CONST robtarget r30AC{targetnumber8} := [" +
            f"[{x:.4f},{y:.4f},{z:.4f}]," +
            f"[{qw:.7f},{qx:.7f},{qy:.7f},{qz:.7f}]," +
            "[-2,-1,1,2]," +
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
        targetnumber8+=1
        
    for ponto in pontos_menos30grausretaAC:
        x=ponto[0]
        y=ponto[1]
        z=ponto[2]
        qw=ponto[6]
        qx=ponto[3]
        qy=ponto[4]
        qz=ponto[5] 
        ficheiro.write(
            f"    CONST robtarget rm30AC{targetnumber9} := [" +
            f"[{x:.4f},{y:.4f},{z:.4f}]," +
            f"[{qw:.7f},{qx:.7f},{qy:.7f},{qz:.7f}]," +
            "[-2,-1,1,2]," +
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
        targetnumber9+=1


    for ponto in pontos_30grausretaBD:
        x=ponto[0]
        y=ponto[1]
        z=ponto[2]
        qw=ponto[6]
        qx=ponto[3]
        qy=ponto[4]
        qz=ponto[5]
        ficheiro.write(
            f"    CONST robtarget r30BD{targetnumber10} := [" +
            f"[{x:.4f},{y:.4f},{z:.4f}]," +
            f"[{qw:.7f},{qx:.7f},{qy:.7f},{qz:.7f}]," +
            "[-2,-1,1,2]," +
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
        targetnumber10+=1
        
    for ponto in pontos_menos30grausretaBD:
        x=ponto[0]
        y=ponto[1]
        z=ponto[2]
        qw=ponto[6]
        qx=ponto[3]
        qy=ponto[4]
        qz=ponto[5]
        ficheiro.write(
            f"    CONST robtarget rm30BD{targetnumber11} := [" +
            f"[{x:.4f},{y:.4f},{z:.4f}]," +
            f"[{qw:.7f},{qx:.7f},{qy:.7f},{qz:.7f}]," +
            "[-2,-1,1,2]," +
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
        targetnumber11+=1








# Main procedure
    ficheiro.write("    PROC main()\n")

    ficheiro.write(f"        MoveJ p1,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"        MoveJ a,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write("        C0BA;  \n")
    ficheiro.write("        C0CD;  \n")
    ficheiro.write("        C30BA; \n")
    ficheiro.write("        C30CD; \n")
    ficheiro.write("        Cm30BA; \n")
    ficheiro.write("        Cm30CD; \n")
    ficheiro.write("        CrAC0; \n")
    ficheiro.write("        CrDB0; \n")
    ficheiro.write("        CrAC30; \n")
    ficheiro.write("        CrDB30; \n")
    ficheiro.write("        CrACm30; \n")
    ficheiro.write("        CrDBm30; \n")
    ficheiro.write(f"        MoveJ a,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"        MoveJ p1,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write("    ENDPROC \n")   
        



# Write path
# The first motion is a joint motion
    targetnumber0=0
    targetnumber1=0
    targetnumber2=0
    targetnumber3=0
    targetnumber4=0
    targetnumber5=0
    targetnumber6=0
    targetnumber7=0
    targetnumber8=0
    targetnumber9=0
    targetnumber10=0
    targetnumber11=0
   #Parte caminho 0 graus
    ficheiro.write("    PROC C0BA() \n")   

    ficheiro.write(f"          MoveJ t0AB0,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"          MoveC t0AB1, t0AB2,v300,z1,{tool}\\WObj:={wobj};\n")

    for ponto in (contorno_escalado0):       
  
        if (targetnumber0>2) and (targetnumber0<(len(contorno_escalado0)-2)): 
            ficheiro.write(
            f"          MoveL t0AB{targetnumber0},v300,z1,{tool}\\WObj:={wobj};\n")        
        
        if (targetnumber0>=(len(contorno_escalado0)-2)): 
            ficheiro.write(
            f"          MoveC t0AB{targetnumber0}, t0AB{targetnumber0+1},v300,z1,{tool}\\WObj:={wobj};\n")
            break        
        targetnumber0+=1

    ficheiro.write("    ENDPROC \n")  

    ficheiro.write("    PROC C0CD() \n")       
    ficheiro.write(f"          MoveJ t0CD0,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"          MoveC t0CD1, t0CD2,v300,z1,{tool}\\WObj:={wobj};\n")
    for ponto in (contorno_escalado1):       
  
        if (targetnumber1>2) and (targetnumber1<(len(contorno_escalado1)-2)):  
            ficheiro.write(
            f"          MoveL t0CD{targetnumber1},v300,z1,{tool}\\WObj:={wobj};\n") 
        if (targetnumber1>=(len(contorno_escalado1)-2)): 
            ficheiro.write(
            f"          MoveC t0CD{targetnumber1}, t0CD{targetnumber1+1},v300,z1,{tool}\\WObj:={wobj};\n")
            break        
     
        targetnumber1+=1
    ficheiro.write("    ENDPROC \n")   



#Parte caminho 30 graus
    ficheiro.write("    PROC C30BA() \n")   
    ficheiro.write(f"          MoveJ t30BA0,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"          MoveC t30BA1, t30BA2,v300,z1,{tool}\\WObj:={wobj};\n")
    for ponto in (pontos_30grausBA):      

        if (targetnumber2>2) and (targetnumber2<(len(pontos_30grausBA)-2)): 
            ficheiro.write(
            f"          MoveL t30BA{targetnumber2},v300,z1,{tool}\\WObj:={wobj};\n")    
        if (targetnumber2>=(len(pontos_30grausBA)-2)): 
            ficheiro.write(
            f"          MoveC t30BA{targetnumber2}, t30BA{targetnumber2+1},v300,z1,{tool}\\WObj:={wobj};\n")
            break          
        targetnumber2+=1
    ficheiro.write("    ENDPROC\n")

    ficheiro.write("    PROC C30CD() \n")   
    ficheiro.write(f"          MoveJ t30CD0,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"          MoveC t30CD1, t30CD2,v300,z1,{tool}\\WObj:={wobj};\n")
    for ponto in (pontos_30grausCD):      

        if (targetnumber3>2) and (targetnumber3<(len(pontos_30grausCD)-2)): 
            ficheiro.write(
            f"          MoveL t30CD{targetnumber3},v300,z1,{tool}\\WObj:={wobj};\n") 
        if (targetnumber3>=(len(pontos_30grausCD)-2)): 
            ficheiro.write(
            f"          MoveC t30CD{targetnumber3}, t30CD{targetnumber3+1},v300,z1,{tool}\\WObj:={wobj};\n")
            break     
        targetnumber3+=1
    ficheiro.write("    ENDPROC \n")         


#Parte caminho menos 30 graus
    ficheiro.write("    PROC Cm30BA() \n")   
    ficheiro.write(f"          MoveJ tm30BA0,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"          MoveC tm30BA1, tm30BA2,v300,z1,{tool}\\WObj:={wobj};\n")
    for ponto in (pontos_menos30grausBA):      
        if (targetnumber4>2) and (targetnumber4<(len(pontos_menos30grausBA)-2)): 
            ficheiro.write(
            f"          MoveL tm30BA{targetnumber4},v300,z1,{tool}\\WObj:={wobj};\n")  
        if (targetnumber4>=(len(pontos_menos30grausBA)-2)): 
            ficheiro.write(
            f"          MoveC tm30BA{targetnumber4}, tm30BA{targetnumber4+1},v300,z1,{tool}\\WObj:={wobj};\n")
            break      
        targetnumber4+=1
    ficheiro.write("    ENDPROC\n")        
    ficheiro.write("    PROC Cm30CD() \n")   
    ficheiro.write(f"          MoveJ tm30CD0,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"          MoveC tm30CD1, tm30CD2,v300,z1,{tool}\\WObj:={wobj};\n")
    for ponto in (pontos_menos30grausCD):      
        if (targetnumber5>2)and (targetnumber5<(len(pontos_menos30grausCD)-2)):  
            ficheiro.write(
            f"          MoveL tm30CD{targetnumber5},v300,z1,{tool}\\WObj:={wobj};\n")    
        if (targetnumber5>=(len(pontos_menos30grausCD)-2)): 
            ficheiro.write(
            f"          MoveC tm30CD{targetnumber5}, tm30CD{targetnumber5+1},v300,z1,{tool}\\WObj:={wobj};\n")
            break    
        targetnumber5+=1
  
    ficheiro.write("    ENDPROC\n")


    ficheiro.write("    PROC CrAC0() \n") 
    ficheiro.write(f"          MoveJ r0AC0,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"          MoveC r0AC1, r0AC2,v300,z1,{tool}\\WObj:={wobj};\n")
    for ponto in (contorno_retaAC):      
        if (targetnumber6>2)and (targetnumber6<(len(contorno_retaAC)-2)):  
            ficheiro.write(
            f"          MoveL r0AC{targetnumber6},v300,z1,{tool}\\WObj:={wobj};\n")    
        if (targetnumber6>=(len(contorno_retaAC)-2)): 
            ficheiro.write(
            f"          MoveC r0AC{targetnumber6}, r0AC{targetnumber6+1},v300,z1,{tool}\\WObj:={wobj};\n")
            break    
        targetnumber6+=1
    ficheiro.write("    ENDPROC\n")

    ficheiro.write("    PROC CrDB0() \n")  
    ficheiro.write(f"          MoveJ r0BD0,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"          MoveC r0BD1, r0BD2,v300,z1,{tool}\\WObj:={wobj};\n")
    for ponto in (contorno_retaBD):      
        if (targetnumber7>2)and (targetnumber7<(len(contorno_retaBD)-2)):  
            ficheiro.write(
            f"          MoveL r0BD{targetnumber7},v300,z1,{tool}\\WObj:={wobj};\n")    
        if (targetnumber7>=(len(contorno_retaBD)-2)): 
            ficheiro.write(
            f"          MoveC r0BD{targetnumber7}, r0BD{targetnumber7+1},v300,z1,{tool}\\WObj:={wobj};\n")
            break    
        targetnumber7+=1
    ficheiro.write("    ENDPROC\n")

    ficheiro.write("    PROC CrAC30() \n")   
    ficheiro.write(f"          MoveJ r30AC0,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"          MoveC r30AC1, r30AC2,v300,z1,{tool}\\WObj:={wobj};\n")
    for ponto in (pontos_30grausretaAC):      
        if (targetnumber8>2)and (targetnumber8<(len(pontos_30grausretaAC)-2)):  
            ficheiro.write(
            f"          MoveL r30AC{targetnumber8},v300,z1,{tool}\\WObj:={wobj};\n")    
        if (targetnumber8>=(len(pontos_30grausretaAC)-2)): 
            ficheiro.write(
            f"          MoveC r30AC{targetnumber8}, r30AC{targetnumber8+1},v300,z1,{tool}\\WObj:={wobj};\n")
            break    
        targetnumber8+=1
    ficheiro.write("    ENDPROC\n")

    ficheiro.write("    PROC CrDB30() \n")   
    ficheiro.write(f"          MoveJ r30BD0,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"          MoveC r30BD1, r30BD2,v300,z1,{tool}\\WObj:={wobj};\n")
    for ponto in (pontos_30grausretaBD):      
        if (targetnumber9>2)and (targetnumber9<(len(pontos_30grausretaBD)-2)):  
            ficheiro.write(
            f"          MoveL r30BD{targetnumber9},v300,z1,{tool}\\WObj:={wobj};\n")    
        if (targetnumber9>=(len(pontos_30grausretaBD)-2)): 
            ficheiro.write(
            f"          MoveC r30BD{targetnumber9}, r30BD{targetnumber9+1},v300,z1,{tool}\\WObj:={wobj};\n")
            break    
        targetnumber9+=1
    ficheiro.write("    ENDPROC\n")

    ficheiro.write("    PROC CrACm30() \n")   
    ficheiro.write(f"          MoveJ rm30AC0,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"          MoveC rm30AC1, rm30AC2,v300,z1,{tool}\\WObj:={wobj};\n")
    for ponto in (pontos_menos30grausretaAC):      
        if (targetnumber10>2)and (targetnumber10<(len(pontos_menos30grausretaAC)-2)):  
            ficheiro.write(
            f"          MoveL rm30AC{targetnumber10},v300,z1,{tool}\\WObj:={wobj};\n")    
        if (targetnumber10>=(len(pontos_menos30grausretaAC)-2)): 
            ficheiro.write(
            f"          MoveC rm30AC{targetnumber10}, rm30AC{targetnumber10+1},v300,z1,{tool}\\WObj:={wobj};\n")
            break    
        targetnumber10+=1 
    ficheiro.write("    ENDPROC\n")

    ficheiro.write("    PROC CrDBm30() \n")
    ficheiro.write(f"          MoveJ rm30BD0,v300,z1,{tool}\\WObj:={wobj};\n")
    ficheiro.write(f"          MoveC rm30BD1, rm30BD2,v300,z1,{tool}\\WObj:={wobj};\n")
    for ponto in (pontos_menos30grausretaBD):      
        if (targetnumber11>2)and (targetnumber11<(len(pontos_menos30grausretaBD)-2)):  
            ficheiro.write(
            f"          MoveL rm30BD{targetnumber11},v300,z1,{tool}\\WObj:={wobj};\n")    
        if (targetnumber11>=(len(pontos_menos30grausretaBD)-2)): 
            ficheiro.write(
            f"          MoveC rm30BD{targetnumber11}, rm30BD{targetnumber11+1},v300,z1,{tool}\\WObj:={wobj};\n")
            break    
        targetnumber11+=1   
    ficheiro.write("    ENDPROC\n")






# End Module
    ficheiro.write("ENDMODULE\n")
# We"re done
    ficheiro.close()
####################FIM RAPID


print("Programa terminado")

