#############      ##################     ################   #############  #####################
###############    ##################     ################   #############  #####################
#######     ####   ######     #######     #####     ######   #############  #####################
#######   ######   ######     #######     #####    ######        ######     #######      ########
#############      ######     #######     ############           ######     #######      ########
###########        ######     #######     #############          ######     #####################
#######            ######     #######     ######   ######        ######     #####################
#######            ######     #######     ######    #####        ######     ########     ########
#######            ##################     ######    ######       ######     ########     ########
#######            ##################     ######     ######      ######     ########     ########

# Bibliotecas necessárias
import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import os.path
from scipy.spatial.transform import Rotation
from tqdm import tqdm
import plotly.graph_objects as go  # Trocou-se a biblioteca original matplotlib
import os

# Configurações iniciais
enablePlot = True
Imagem2 = False
Imagem3 = False
Imagem4 = False

BAG_FILE = "Bag_Scan_Porta.bag"

# Define output directory relative to this script's location
output_dir = os.path.join(os.path.dirname(__file__), "GeneratedFiles_OCVPorta_25")
os.makedirs(output_dir, exist_ok=True)

def set_axes_equal(ax):
    """Make axes of 3D plot have equal scale so that spheres appear as spheres, cubes as cubes, etc.
    Input ax: a matplotlib axis, e.g., as output from plt.gca()."""
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
    plot_radius = 0.5 * max([x_range, y_range, z_range])
    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


"""
def criar_pipeline():
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
    return pipeline
"""

try:
    pipeline = rs.pipeline()
    config = rs.config()
    rs.config.enable_device_from_file(config, BAG_FILE)
    config.enable_stream(rs.stream.depth, rs.format.z16, 6)  # 16 bits = Z16, 6 FPS
    pipeline.start(config)
    # Cria um objeto colorizer para colorir o fluxo de profundidade, se necessário
    colorizer = rs.colorizer()
    # Define variáveis para o filtro espacial
    spatial_magnitude = 2  # Magnitude do filtro espacial
    spatial_smooth_alpha = 0.3  # Suavização do filtro
    spatial_smooth_delta = 30  # Delta do filtro espacial
    spatial_holes_fill = 1  # Preenchimento de buracos do fluxo
    # Cria filtro temporal (para suavizar a profundidade ao longo do tempo)
    filter_temporal = rs.temporal_filter(smooth_alpha=0.6, smooth_delta=50, persistence_control=3)
    # Número de frames a serem considerados para suavização
    nr_frames_a_considerar = 20
    # Inicializando uma imagem de pré-processamento de profundidade com zeros
    #image_part_pre_trat = np.zeros((720, 1280), dtype=np.uint16)
    # Streaming loop
    # While True:
    # Cria um array de 720 por 1080 e aguarda por um conjunto de frames de profundidade
    image_part_pre_trat = np.empty((720, 1280, 0), dtype=np.uint8)
    image_standardeviationP = np.empty((720, 1280), dtype=np.uint16)
    image_maxP = np.empty((720, 1280), dtype=np.uint16)
    image_minP = np.empty((720, 1280), dtype=np.uint16)
    image_max_deviationP = np.empty((720, 1280), dtype=np.uint16)
    image_min_deviationP = np.empty((720, 1280), dtype=np.uint16)

    # Iterando sobre os quadros que serão considerados para suavização
    print("A recolher frames")
    for i in tqdm(range(nr_frames_a_considerar)):
        frames = pipeline.wait_for_frames()  # Captura dos frames
        depth_frame = frames.get_depth_frame()  # Obtem o frame de profundidade
        spatial = rs.spatial_filter()  # Cria filtro espacial para processar a profundidade
        depth = spatial.process(depth_frame)  # Processa a profundidade com o filtro espacial
        spatial.set_option(rs.option.filter_magnitude, spatial_magnitude)  # Ajusta os parâmetros do filtro espacial
        spatial.set_option(rs.option.filter_smooth_alpha, spatial_smooth_alpha)
        spatial.set_option(rs.option.filter_smooth_delta, spatial_smooth_delta)
        depth = spatial.process(depth)  # Replica o filtro com novas configurações
        spatial.set_option(rs.option.holes_fill,
                           spatial_holes_fill)  # Preenche os buracos (valores em falta) no fluxo de profundidade
        depth = spatial.process(depth)
        filtered_depth = filter_temporal.process(
            depth)  # Aplica o filtro temporal para suavizar o fluxo de profundidade
        depth_image = np.asanyarray(
            filtered_depth.get_data())  # Converte os dados de profundidade filtrados para um array do Numpy
        depth_image_no_filter = np.asanyarray(depth_frame.get_data())  # Profundidade sem filtro

        image_part_pre_trat = np.dstack(
            [image_part_pre_trat, depth_image])  # Empilha os frames ao longo do eixo 2 (profundidade)

        # Debug
        depth_color_frame = colorizer.colorize(
            filtered_depth)  # colorizer.colorize() é usado para aplicar um mapa de cores à imagem de profundidade, neste caso o "JET" mapeia os valores de profundidade para cores azul (profundidade mais próxima) e vermelho (profundidade mais distante)
        depth_color_image = np.asanyarray(
            depth_color_frame.get_data())  # np.asanyarray() é usado para converter os dados da imagem colorida em arrays do tipo numpy

    image_rearenged = np.empty((720, 1280), dtype=np.uint16)  # Cria uma nova imagem para reorganizar a profundidade
    # for i in tqdm(range(10000)):
    print("A calcular médias de frames...")
    # Reorganiza a imagem acumulada ao longo dos frames (pós-processamento)
    for l in tqdm(range(1280)):  # Precorre cada coluna da imagem (1280 colunas)
        for k in range(720):  # Precorre cada linha da imagem (720 linhas)
            zzs = []
            zzs = image_part_pre_trat[
                k, l, :]  # Obtem os valores de profundidade ao longo dos frames para o pixeis (k, l)
            zzs_nozeros = zzs[zzs != 0]  # Calcula a média dos valores, ignorando os zeros
            zzs_aceptable1 = zzs_nozeros[
                zzs_nozeros < 1400]  # Ignora valores acima de 1400 mm (1.4 metros) e abaixo de 500 mm (0.5 metros)
            zzs_aceptable = zzs_aceptable1[zzs_aceptable1 > 500]
            # if (l==857) and (k==58):
            #    print("a")
            if len(zzs_aceptable) < 5:  # If no non-zero values are present
                image_rearenged[k, l] = np.uint16(
                    0)  # Se a média for zero, o valor final é zero (sem profundidade válida)
            else:
                mediaz = np.mean(zzs_aceptable)
                dis = [(z, abs(z - mediaz)) for z in
                       zzs_aceptable]  # Calcula a diferença entre os valores de profundidade e a média
                dis.sort(key=lambda x: x[1])  # Ordena pela distância da média
                n = int(round(
                    len(zzs_aceptable) * 0.7))  # Seleciona os 'n' valores mais próximos da média (70% dos valores)
                image_rearenged[k, l] = np.uint16(
                    round(np.mean([x[0] for x in dis])))  # Calcula a média dos valores selecionados para o pixel (k, l)
                # if l==k==69 and np.all(zzs!=0):
                #    print("med:",np.uint16(np.mean(z_sel)) )
                image_standardeviationP[k, l] = np.uint16(round(np.std(zzs_aceptable)))
                image_maxP[k, l] = np.uint16(round(np.max(zzs_aceptable)))
                image_minP[k, l] = np.uint16(round(np.min(zzs_aceptable)))
                desvio = np.abs(zzs_aceptable - image_rearenged[k, l])
                image_max_deviationP[k, l] = np.uint16(round(np.max(desvio)))
                image_min_deviationP[k, l] = np.uint16(round(np.min(desvio)))
    print("Médias de frames calculados.")

    # Gráfico grafico_porta_profundidade [Descomentar para ver gráfico]

    # Plot da imagem de profundidade
    if enablePlot:
        # Criar coordenadas X e Y para cada pixel
        Y, X = np.indices(image_rearenged.shape)
        Z = image_rearenged

        # Achatar os arrays para 1D
        X = X.ravel()
        Y = Y.ravel()
        Z = Z.ravel()

        step = 5 # plot 1 out of every 5 points
        X = X[::step]
        Y = Y[::step]
        Z = Z[::step]

        # Filter out zero points
        valid_mask = Z > 0
        X_valid = X[valid_mask]
        Y_valid = Y[valid_mask]
        Z_valid = Z[valid_mask]

        # Invert Z axis
        Z_valid = -Z_valid

        print("Total de pontos válidos:", len(Z_valid))
        if len(Z_valid) == 0:
            print("AVISO: Nenhum ponto válido para plotar no gráfico 3D!")
        else:
            print("Z range:", Z_valid.min(), Z_valid.max())
            if len(Z_valid) > 10000:
                print(Z_valid[10000], X_valid[10000], Y_valid[10000])

            # Criar scatter plot 3D
            fig = go.Figure(data=[go.Scatter3d(
                x=X_valid, y=Y_valid, z=Z_valid,
                mode='markers',
                marker=dict(
                    size=3,
                    color=Z_valid,
                    colorscale='Inferno',
                    colorbar=dict(title='Profundidade'),
                    opacity=0.7
                )
            )])

            fig.update_layout(
                scene=dict(
                    xaxis_title='X',
                    yaxis_title='Y',
                    zaxis_title='Z (invertido)',
                    xaxis=dict(range=[X_valid.min()-50, X_valid.max()+50]),
                    zaxis=dict(range=[Z_valid.min()-10, Z_valid.max()+10]),
                    camera=dict(eye=dict(x=1.5, y=1.5, z=0.5)),
                    aspectmode='manual',
                    aspectratio=dict(x=2, y=1, z=0.5)
                ),
                title='Porta - Profundidade (Z invertido)',
                autosize=True
            )

            fig.write_html(os.path.join(output_dir, "grafico_porta_profundidade.html"), config=dict(responsive=True)) # GRAFICO OK


    # desvioPadraoMedioGeral=np.mean(image_standardeviationP)
    desvioMedMascara = (image_standardeviationP > 0.000) & (image_standardeviationP < 1000)
    valores_filtradosMed = image_standardeviationP[desvioMedMascara]
    desvioPadraoMedioGeral = np.mean(valores_filtradosMed)

    # desvioPadraoMaximoGeral=np.mean(image_max_deviationP)
    desvioMaxMascara = (image_max_deviationP > 0.000) & (image_max_deviationP < 1000)
    valores_filtradosMax = image_standardeviationP[desvioMaxMascara]
    desvioPadraoMaximoGeral = np.max(valores_filtradosMax)

    # desvioPadraoMinimoGeral=np.mean(image_min_deviationP)
    desvioMinMascara = (image_min_deviationP > 0.000) & (image_min_deviationP < 1000)
    valores_filtradosMin = image_standardeviationP[desvioMinMascara]
    desvioPadraoMinimoGeral = np.min(valores_filtradosMin)

    print("Desvio Padrão Médio geral Porta=", desvioPadraoMedioGeral)
    # print("Menor desvio geral Porta=", desvioPadraoMinimoGeral)
    # print("Maior desvio geral Porta=", desvioPadraoMaximoGeral)

    # Gráfico grafico_porta_desvio_padrao [Descomentar para ver gráfico]
    """
    # Plot do desvio padrão da profundidade
    if enablePlot:
        # Criar coordenadas X e Y para cada pixel
        Y, X = np.indices(image_standardeviationP.shape)  # Y: linhas, X: colunas
        Z = image_standardeviationP

        # Achatar os arrays para 1D
        X = X.ravel()
        Y = Y.ravel()
        Z = Z.ravel()

        # Filtrar apenas pontos válidos (Z > 0)
        mascara_validos = Z > 0
        X_valid = X[mascara_validos]
        Y_valid = Y[mascara_validos]
        Z_valid = -Z[mascara_validos]  # Invert Z axis

        print("Total de pontos válidos para o gráfico de desvio padrão:", len(Z_valid))

        # Criar scatter plot 3D apenas com pontos válidos
        fig = go.Figure(data=[go.Scatter3d(
            x=X_valid, y=Y_valid, z=Z_valid,
            mode='markers',
            marker=dict(size=2, color=Z_valid, colorscale='Inferno', colorbar=dict(title='Profundidade'))
        )])
        fig.update_layout(
            scene=dict(
                xaxis_title='X',
                yaxis_title='Y',
                zaxis_title='Y (invertido)',
                xaxis=dict(range=[X_valid.min()-50, X_valid.max()+50]),
                camera=dict(eye=dict(x=0, y=2, z=0.5)),
                aspectmode='manual',
                aspectratio=dict(x=16, y=9, z=6)
            ),
            title='Porta - Desvio Padrão (Z invertido)'
        )
        fig.write_html(os.path.join(output_dir, "grafico_porta_desvio_padrao.html")) # GRAFICO OK
    """

    cv2.waitKey(0)  # Espera indefinidamente até que qualquer tecla seja pressionada
    normed = cv2.normalize(image_rearenged, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    # O método cv2.normalize pega nos valores de profundidade da image_rearenged e vai mapeá-los para o intervalo de 0 a 255
    # Exibe a imagem de profundidade colorida em uma janela chamada "Depth Stream"
    if (Imagem2):
        cv2.imshow("1 - Escala cinzetnos", normed)
        cv2.waitKey(0)  # Espera indefinidamente até que qualquer tecla seja pressionada
    # Aplica um mapa de cores 'JET' à imagem normalizada.
    # Cria uma visualização colorida da imagem de profundidade, usando uma paleta de cores para representar diferentes distâncias.
    colora = cv2.applyColorMap(normed, cv2.COLORMAP_JET)
    # Exibe a imagem de profundidade colorida em uma janela chamada "Depth Stream"
    if (Imagem3):
        cv2.imshow("2 - Depth StreamB", colora)
        cv2.waitKey(0)  # Espera indefinidamente até que qualquer tecla seja pressionada

    # Calcula a menor distância (profundidade) na imagem, ignorando os valores zero
    # A função np.min encontra o valor mínimo da imagem de profundidade que não seja zero (ignorando assim os valores que seriam zero)
    min_dist = np.min(image_rearenged[image_rearenged != 0])
    # Cria uma nova imagem em branco (preenchida com zeros), com a mesma forma da imagem de profundidade
    image_part_uint8 = np.zeros(image_rearenged.shape, dtype=np.uint8)
    # Cria uma máscara de áreas com profundidade próxima à mínima, criando uma "zona de interesse".
    image_part_uint8[(image_rearenged <= min_dist + 100)] = 255
    # Define o pixels com valor 0 (sem profundidade) como 0 na imagem
    image_part_uint8[(image_rearenged == 0)] = 0
    # Cria um elemento estruturante em forma de elipse (usado para operações morfológicas)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    # Aplica a erosão morfológica à imagem para reduzir os ruídos e as pequenas áreas de alto valor
    image_part_uint8 = cv2.erode(image_part_uint8, kernel, iterations=1)
    # Aplica a dilatação morfológica à imagem, que expande as regiões brancas após a erosão
    image_part_uint8 = cv2.dilate(image_part_uint8, kernel, iterations=1)

    # Mostra a imagem após as operações morfológicas em uma nova janela chamada "Depth Stream50"
    if (Imagem4):
        cv2.imshow("3 - Binarizado [preto ou branco]", image_part_uint8)
        cv2.waitKey(0)  # Espera indefinidamente até que qualquer tecla seja pressionada

    # Aplica um mapa de cores para facilitar a visualização da profundidade, usando o colormap 'JET'.
    depth_color_frame = colorizer.colorize(
        filtered_depth)  # colorizer.colorize() é usado para aplicar um mapa de cores à imagem de profundidade, neste caso o "JET" mapeia os valores de profundidade para cores azul (profundidade mais próxima) e vermelho (profundidade mais distante)
    # Aplica o mapa de cores 'JET' diretamente ao quadro original sem nenhum processamento adicional.
    depth_color_frame_no_filter = colorizer.colorize(depth_frame)

    # Converte os frames de profundidade coloridos para arrays numpy, para poderem ser exibidos no OpenCv
    depth_color_image = np.asanyarray(
        depth_color_frame.get_data())  # np.asanyarray() é usado para converter os dados da imagem colorida em arrays do tipo numpy
    depth_color_image_no_filter = np.asanyarray(depth_color_frame_no_filter.get_data())


finally:
    pass

"""ENCONTRAR CONTORNOS DA PORTA"""
# Encontrando os contornos na imagem binarizada (image_part)
# cv2.findContours detecta os contornos na imagem e retorna uma lista de contornos.
contorno_bag, _ = cv2.findContours(image_part_uint8, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)  # cv2.CHAIN_APPROX_SIMPLE simplifica os contornos, removendo pontos redundantes (útil para reduzir a quantidade de dados).
# Seleciona o maior contorno encontrado (o que possui a maior área)
maior_contorno_bag = max(contorno_bag,
                         key=cv2.contourArea)  # cv2.contourArea calcula a área de cada contorno. max() seleciona o contorno de maior área.
maior_contorno_bag = maior_contorno_bag.reshape(-1, 2)
maior_contorno_bag = np.array(maior_contorno_bag)
# Inicializa uma imagem preta (com fundo preto) para desenhar os contornos encontrados
contorno_bag_image = np.zeros(depth_color_image.shape, dtype=np.uint8)

i = 0
contornoComSD = np.array([])
for ponto in maior_contorno_bag:
    k = ponto[1]
    l = ponto[0]
    standardeviationContour = image_standardeviationP[k, l]
    point = np.array([l, k, standardeviationContour])
    if i == 0:
        contornoComSD = point
    else:
        contornoComSD = np.vstack((contornoComSD, point))
    i += 1

# Gráfico grafico_contorno_azim0 [Descomentar para ver gráfico]

# Desenha o maior contorno na imagem preta (contorno_bag_image) com a cor verde (0, 255, 0) e espessura de 2 pixels
if enablePlot:
    X = contornoComSD[:, 0]
    Y = contornoComSD[:, 1]
    Z = -contornoComSD[:, 2]  # Invert Z axis

    # Optionally rescale Z for better visualization (e.g., to [0, 1])
    # Z = (Z - Z.min()) / (Z.max() - Z.min())

    fig = go.Figure(data=[go.Scatter3d(
        x=X, y=Y, z=Z,
        mode='markers',
        marker=dict(size=2, color=Z, colorscale='Inferno_r', colorbar=dict(title='Profundidade'))
    )])
    fig.update_layout(
        scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z (invertido)',
            xaxis=dict(range=[X.min()-50, X.max()+50]),
            camera=dict(eye=dict(x=1.5, y=1.5, z=0.5)),
            aspectmode='manual',
            aspectratio=dict(x=16, y=9, z=6)
        ),
        autosize=True,
        title='Contorno SD - Azimute 0 (Z invertido)'
    )
    fig.write_html(os.path.join(output_dir, "grafico_contorno_azim0.html"), config=dict(responsive=True)) # GRAFICO OK


desvioPadraoMaxContorno = np.max(contornoComSD[:, 2])
desvioPadraoMinContorno = np.min(contornoComSD[:, 2])
desvioPadraoMedContorno = np.mean(contornoComSD[:, 2])

print("Desvio Padrão Máximo Contorno =", desvioPadraoMaxContorno)
print("Desvio Padrão Mínimo Contorno =", desvioPadraoMinContorno)
print("Desvio Padrão Médio Contorno =", desvioPadraoMedContorno)

valores_contorno = []
for ponto in maior_contorno_bag:
    k = ponto[1]
    l = ponto[0]
    valor = image_rearenged[k, l]
    if valor != 0:
        valores_contorno.append(valor)

if len(valores_contorno) > 0:
    desvioPadraoProfundidadeContorno = np.std(valores_contorno)
    print("Desvio Padrão da Profundidade apenas no contorno =", desvioPadraoProfundidadeContorno)
else:
    print("Nenhum valor de profundidade válido no contorno.")

# Save arrays to .npy files
np.save(os.path.join(output_dir, "arraySCANPorta.npy"), image_rearenged)
print(f"Array de média de frames da porta gravada no ficheiro {os.path.join(output_dir, 'arraySCANPorta.npy')}.")
np.save(os.path.join(output_dir, "maiorContornoPorta.npy"), maior_contorno_bag)
print(f"Array de média de frames da porta gravada no ficheiro {os.path.join(output_dir, 'maiorContornoPorta.npy')}.")
np.save(os.path.join(output_dir, "contorno_bag_image.npy"), contorno_bag_image)
print(f"contorno_bag_image gravada no ficheiro {os.path.join(output_dir, 'contorno_bag_image.npy')}.")
np.save(os.path.join(output_dir, "arrayMaxPorta.npy"), image_maxP)
np.save(os.path.join(output_dir, "arrayMinPorta.npy"), image_minP)
np.save(os.path.join(output_dir, "arrayDesvioPadraoPorta.npy"), image_standardeviationP)


# Gráfico 2D do contorno
def plot_contorno_2d(contorno, filename="contorno_2d.html", titulo="Contorno 2D"):
    import plotly.graph_objects as go
    X = contorno[:, 0]
    Y = contorno[:, 1]
    fig = go.Figure(data=[go.Scatter(
        x=X, y=Y,
        mode='markers+lines',
        marker=dict(size=3, color='blue'),
        line=dict(color='red', width=1)
    )])
    fig.update_layout(
        xaxis_title='X',
        yaxis_title='Y',
        title=titulo,
        width=1280,
        height=720
    )
    # Always save in the output_dir folder
    full_path = os.path.join(output_dir, filename)
    fig.write_html(full_path)
    print(f"Gráfico 2D do contorno salvo em {full_path}")  # GRAFICO OK


# Exemplo de uso após encontrar o maior contorno:
if enablePlot:
    try:
        plot_contorno_2d(maior_contorno_bag, filename="contorno_maior_2d.html", titulo="Maior Contorno 2D da Porta")
    except Exception as e:
        print("Erro ao criar gráfico 2D do contorno:", e)

print("OCVPorta_25 terminado")