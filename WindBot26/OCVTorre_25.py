################    ###################     ##################      ##################   ###############
################    ###################     ##################      ##################   ###############
################    ######      #######     ######      ######      ######      ######   ###############
######          ######      #######     ##################      ##################   ######
######          ######      #######     ################        ################     ##########
######          ######      #######     ##############          ##############       ##########
######          ######      #######     ######   ######         ######   ######      ######
######          ######      #######     ######     ######       ######     ######    ######
######          ######      #######     ######       #####      ######       #####   ###############
######          ###################     ######        #####     ######        #####  ###############
######          ###################     ######         #####    ######         ##### ###############

# Bibliotecas necessárias
import pyrealsense2 as rs
import numpy as np
# import cv2
# import argparse
# import os.path
# from scipy.spatial.transform import Rotation
from tqdm import tqdm
import plotly.graph_objects as go
import os

# Configurações iniciais
enablePlot = True
BAG_FILE = "Bag_Scan_Torre.bag"

# Define output directory relative to this script's location
output_dir = os.path.join(os.path.dirname(__file__), "GeneratedFiles_OCVTorre_25")
os.makedirs(output_dir, exist_ok=True)

try:
    pipelineT = rs.pipeline()
    configT = rs.config()
    rs.config.enable_device_from_file(configT, BAG_FILE)
    configT.enable_stream(rs.stream.depth, rs.format.z16, 6)  # 16 bits = Z16, 6 FPS
    pipelineT.start(configT)
    colorizer = rs.colorizer()  # Cria um objeto colorizer para colorir o fluxo de profundidade, se necessário
    spatial_magnitude = 2  # Magnitude do filtro espacial
    spatial_smooth_alpha = 0.3  # Suavização do filtro
    spatial_smooth_delta = 30  # Delta do filtro espacial
    spatial_holes_fill = 0  # Preenchimento de buracos do fluxo
    filter_temporal = rs.temporal_filter(smooth_alpha=0.6, smooth_delta=50, persistence_control=3)
    nr_frames_a_considerar = 20
    image_part_pre_tratT = np.zeros((720, 1280),
                                    dtype=np.uint16)  # Inicializando uma imagem de pré-processamento de profundidade com zeros

    # Streaming loop
    # While True:
    # Cria um array de 720 por 1080 e aguarda por um conjunto de frames de profundidade
    image_part_pre_tratT = np.empty((720, 1280, 0), dtype=np.uint8)

    # Iterando sobre os quadros que serão considerados para suavização
    print("A recolher frames")
    for i in tqdm(range(nr_frames_a_considerar)):
        framesT = pipelineT.wait_for_frames()
        depth_frameT = framesT.get_depth_frame()  # Obtem o frame de profundidade
        spatial = rs.spatial_filter()  # Cria filtro espacial para processar a profundidade
        depthT = spatial.process(depth_frameT)  # Processa a profundidade com o filtro espacial
        spatial.set_option(rs.option.filter_magnitude, spatial_magnitude)
        spatial.set_option(rs.option.filter_smooth_alpha, spatial_smooth_alpha)
        spatial.set_option(rs.option.filter_smooth_delta, spatial_smooth_delta)
        depthT = spatial.process(depthT)  # Replica o filtro com novas configurações
        spatial.set_option(rs.option.holes_fill,
                           spatial_holes_fill)  # Preenche os buracos (valores em falta) no fluxo de profundidade
        depthT = spatial.process(depthT)
        filtered_depthT = filter_temporal.process(
            depthT)  # Aplica o filtro temporal para suavizar o fluxo de profundidade
        depth_imageT = np.asanyarray(
            filtered_depthT.get_data())  # Converte os dados de profundidade filtrados para um array do Numpy
        depth_image_no_filterT = np.asanyarray(depth_frameT.get_data())  # Profundidade sem filtro

        image_part_pre_tratT = np.dstack(
            [image_part_pre_tratT, depth_imageT])  # Empilha os frames ao longo do eixo 2 (profundidade)

        # Debug
        # depth_color_frameT = colorizer.colorize(filtered_depthT) # colorizer.colorize() é usado para aplicar um mapa de cores à imagem de profundidade, neste caso o "JET" mapeia os valores de profundidade para cores azul (profundidade mais próxima) e vermelho (profundidade mais distante)
        # depth_color_imageT = np.asanyarray(depth_color_frameT.get_data()) # np.asanyarray() é usado para converter os dados da imagem colorida em arrays do tipo numpy

    # Cria uma nova imagem para reorganizar a profundidade
    image_rearengedT = np.empty((720, 1280), dtype=np.uint16)
    image_standardeviation = np.empty((720, 1280), dtype=np.uint16)
    image_max = np.empty((720, 1280), dtype=np.uint16)
    image_min = np.empty((720, 1280), dtype=np.uint16)
    image_max_deviation = np.empty((720, 1280), dtype=np.uint16)
    image_min_deviation = np.empty((720, 1280), dtype=np.uint16)

    print("A calcular médias de frames TORRE...")
    # Reorganiza a imagem acumulada ao longo dos frames (pós-processamento)
    for lT in tqdm(range(1280)):  # Precorre cada coluna da imagem (1280 colunas)
        for kT in range(720):  # Precorre cada linha da imagem (720 linhas)
            zzsT = []
            zzsT = image_part_pre_tratT[
                kT, lT, :]  # Obtem os valores de profundidade ao longo dos frames para o pixeis (k, l)
            zzs_nozerosT = zzsT[zzsT != 0]  # Calcula a média dos valores, ignorando os zeros
            zzs_aceptable1T = zzs_nozerosT[zzs_nozerosT < 1400]  # Valores aceitáveis (entre 500 mm e 2000 mm)
            zzs_aceptableT = zzs_aceptable1T[zzs_aceptable1T > 500]
            # if (l==857) and (k==58):
            #    print("a")
            if len(zzs_aceptableT) < 5:  # If no non-zero values are present
                image_rearengedT[kT, lT] = np.uint16(
                    0)  # Se a média for zero, o valor final é zero (sem profundidade válida)
            else:
                mediazT = np.mean(zzs_aceptableT)
                disT = [(zT, abs(zT - mediazT)) for zT in
                        zzs_aceptableT]  # Calcula a diferença entre os valores de profundidade e a média
                disT.sort(key=lambda xT: xT[1])  # Ordena pela distância da média
                nT = int(round(
                    len(zzs_aceptableT) * 0.7))  # Seleciona os 'n' valores mais próximos da média (70% dos valores)

                image_rearengedT[kT, lT] = np.uint16(round(
                    np.mean([xT[0] for xT in disT])))  # Calcula a média dos valores selecionados para o pixel (k, l)
                image_standardeviation[kT, lT] = np.uint16(round(np.std(zzs_aceptableT)))
                image_max[kT, lT] = np.uint16(round(np.max(zzs_aceptableT)))
                image_min[kT, lT] = np.uint16(round(np.min(zzs_aceptableT)))
                desvio = np.abs(zzs_aceptableT - image_rearengedT[kT, lT])
                image_max_deviation[kT, lT] = np.uint16(round(np.max(desvio)))
                image_min_deviation[kT, lT] = np.uint16(round(np.min(desvio)))

    print("Médias de frames calculados(TORRE).")
    np.save(os.path.join(output_dir, "arraySCANTorre.npy"), image_rearengedT)
    np.save(os.path.join(output_dir, "arrayMaxTorre.npy"), image_max)
    np.save(os.path.join(output_dir, "arrayMinTorre.npy"), image_min)
    np.save(os.path.join(output_dir, "arrayDesvioPadraoTorre.npy"), image_standardeviation)
    np.save(os.path.join(output_dir, "arrayDesvioMaximoTorre.npy"), image_max_deviation)
    np.save(os.path.join(output_dir, "arrayDesvioMinimoTorre.npy"), image_min_deviation)

    desvioPadraoMedioGeral = np.mean(image_standardeviation)
    # desvioPadraoMaximoGeral=np.mean(image_max_deviation)
    # desvioPadraoMinimoGeral=np.mean(image_min_deviation)
    print("Desvio Padrão Médio geral=", desvioPadraoMedioGeral)
    # print("Desvio Padrão Mínimo geral=", desvioPadraoMinimoGeral)
    # print("Desvio Padrão Máximo geral=", desvioPadraoMaximoGeral)
    print("Array de média de frames da torre gravada no ficheiro arraySCANTorre.npy.")

    # Gráfico grafico_torre_desvio_padrao [Descomentar para ver o gráfico]
    """
    if enablePlot==True:
        Y, X = np.indices(image_standardeviation.shape)
        Z = image_standardeviation
        X = X.ravel()
        Y = Y.ravel()
        Z = Z.ravel()
        mascara_validos = Z > 0
        X_valid = X[mascara_validos]
        Y_valid = Y[mascara_validos]
        Z_valid = Z[mascara_validos]
        print(f"Desvio Padrão Torre: pontos válidos = {len(Z_valid)}")
        np.save("debug_X_valid_torre_desvio_padrao.npy", X_valid)
        np.save("debug_Y_valid_torre_desvio_padrao.npy", Y_valid)
        np.save("debug_Z_valid_torre_desvio_padrao.npy", Z_valid)
        fig = go.Figure(data=[go.Scatter3d(
            x=X_valid, y=Y_valid, z=Z_valid,
            mode='markers',
            marker=dict(size=2, color=Z_valid, colorscale='Inferno', colorbar=dict(title='Desvio Padrão-Torre'))
        )])
        fig.update_layout(
            scene=dict(
                xaxis_title='X',
                yaxis_title='Y',
                zaxis_title='Desvio Padrão-Torre',
                xaxis=dict(range=[0,1280]),
                yaxis=dict(range=[0,720]),
                zaxis=dict(range=[np.min(Z_valid), np.max(Z_valid)]),
                camera=dict(eye=dict(x=0, y=2, z=0.5))
            ),
            autosize=True,
            title='Desvio Padrão Torre'
        )
        fig.write_html("grafico_torre_desvio_padrao.html", config=dict(responsive=True))
    """

    # Gráfico grafico_torre_desvio_padrao_azim0 [Descomentar para ver o gráfico]
    """
    if enablePlot==True:
        Y, X = np.indices(image_standardeviation.shape)
        Z = image_standardeviation
        X = X.ravel()
        Y = Y.ravel()
        Z = Z.ravel()
        mascara_validos = Z > 0
        X_valid = X[mascara_validos]
        Y_valid = Y[mascara_validos]
        Z_valid = Z[mascara_validos]
        print(f"Desvio Padrão Torre Azimute 0: pontos válidos = {len(Z_valid)}")
        np.save("debug_X_valid_torre_desvio_padrao_azim0.npy", X_valid)
        np.save("debug_Y_valid_torre_desvio_padrao_azim0.npy", Y_valid)
        np.save("debug_Z_valid_torre_desvio_padrao_azim0.npy", Z_valid)
        fig = go.Figure(data=[go.Scatter3d(
            x=X_valid, y=Y_valid, z=Z_valid,
            mode='markers',
            marker=dict(size=2, color=Z_valid, colorscale='Inferno', colorbar=dict(title='Desvio Padrão-Torre'))
        )])
        fig.update_layout(
            scene=dict(
                xaxis_title='X',
                yaxis_title='Y',
                zaxis_title='Desvio Padrão-Torre',
                xaxis=dict(range=[0,1280]),
                yaxis=dict(range=[0,720]),
                zaxis=dict(range=[np.min(Z_valid), np.max(Z_valid)]),
                camera=dict(eye=dict(x=2, y=0, z=0.5))
            ),
            autosize=True,
            title='Desvio Padrão Torre Azimute 0'
        )
        fig.write_html("grafico_torre_desvio_padrao_azim0.html", config=dict(responsive=True))
    """

    # Criar coordenadas X e Y para cada pixel
    Y, X = np.indices(image_standardeviation.shape)
    Z = image_rearengedT
    X = X.ravel()
    Y = Y.ravel()
    Z = Z.ravel()
    mascara_validos = Z > 0
    X_valid = X[mascara_validos]
    Y_valid = Y[mascara_validos]
    Z_valid = Z[mascara_validos]
    print(f"Superfície Torre: pontos válidos = {len(Z_valid)}")
    np.save(os.path.join(output_dir, "debug_X_valid_superficie_torre.npy"), X_valid)
    np.save(os.path.join(output_dir, "debug_Y_valid_superficie_torre.npy"), Y_valid)
    np.save(os.path.join(output_dir, "debug_Z_valid_superficie_torre.npy"), Z_valid)
    # Amostra de 1000 pontos para teste visual
    n_amostra = min(1000, len(Z_valid))
    if n_amostra > 0:
        idx = np.random.choice(len(Z_valid), n_amostra, replace=False)
        X_sample = X_valid[idx]
        Y_sample = Y_valid[idx]
        Z_sample = Z_valid[idx]
        Z_sample = -Z_sample  # Inverte os valores de Z para melhor visualização
        fig = go.Figure(data=[go.Scatter3d(
            x=X_sample, y=Y_sample, z=Z_sample,
            mode='markers',
            marker=dict(size=4, color=Z_sample, colorscale='Inferno', colorbar=dict(title='Profundidade Torre'))
        )])

        fig.update_layout(
            scene=dict(
                xaxis_title='X',
                yaxis_title='Y',
                zaxis_title='Profundidade Torre',
                # xaxis=dict(range=[0,1280]),
                # yaxis=dict(range=[0,720]),
                # zaxis=dict(range=[np.min(Z_sample), np.max(Z_sample)]),
                camera=dict(eye=dict(x=0, y=2, z=0.5))
            ),
            autosize=True,
            title='Superfície 3D da Torre (Amostra)'
        )
        fig.write_html(os.path.join(output_dir, "grafico_superficie_torre_amostra.html"), config=dict(responsive=True))
        # Gráfico 2D simples para diagnóstico
        fig2d = go.Figure(data=[go.Scatter(
            x=X_sample,
            y=Z_sample,
            mode='markers',
            marker=dict(size=4, color=-Z_sample, colorscale='Inferno', colorbar=dict(title='Profundidade Torre'))
        )])
        fig2d.update_layout(
            xaxis_title='X',
            yaxis_title='Profundidade Torre',
            title='Diagnóstico 2D X vs Profundidade'
        )
        fig2d.write_html(os.path.join(output_dir, "grafico_superficie_torre_2d.html"))
    else:
        print("Nenhum ponto válido para amostra!")

    # Gráfico grafico_torre_desvio_padrao_azim90 [Descomentar para ver o gráfico]
    """
    if enablePlot==True:
        Y, X = np.indices(image_standardeviation.shape)
        Z = image_standardeviation
        X = X.ravel()
        Y = Y.ravel()
        Z = Z.ravel()
        mascara_validos = Z > 0
        X_valid = X[mascara_validos]
        Y_valid = Y[mascara_validos]
        Z_valid = Z[mascara_validos]
        print(f"Desvio Padrão Torre Azimute 90: pontos válidos = {len(Z_valid)}")
        np.save("debug_X_valid_torre_desvio_padrao_azim90.npy", X_valid)
        np.save("debug_Y_valid_torre_desvio_padrao_azim90.npy", Y_valid)
        np.save("debug_Z_valid_torre_desvio_padrao_azim90.npy", Z_valid)
        fig = go.Figure(data=[go.Scatter3d(
            x=X_valid, y=Y_valid, z=Z_valid,
            mode='markers',
            marker=dict(size=2, color=Z_valid, colorscale='Inferno', colorbar=dict(title='Desvio Padrão-Torre'))
        )])
        fig.update_layout(
            scene=dict(
                xaxis_title='X',
                yaxis_title='Y',
                zaxis_title='Desvio Padrão-Torre',
                xaxis=dict(range=[0,1280]),
                yaxis=dict(range=[0,720]),
                zaxis=dict(range=[np.min(Z_valid), np.max(Z_valid)]),
                camera=dict(eye=dict(x=0, y=2, z=0.5))
            ),
            autosize=True,
            title='Desvio Padrão Torre Azimute 90'
        )
        fig.write_html("grafico_torre_desvio_padrao_azim90.html", config=dict(responsive=True))
        """

finally:
    pass

print("OCVTorre_25 terminado")