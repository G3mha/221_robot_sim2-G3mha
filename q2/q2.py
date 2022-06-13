import cv2


def encontra_figuras(img_bgr):
    """
    Cria e retorna uma nova imagem BGR com os
    pontos de fuga desenhados.

    Entrada:
    - img_bgr: imagem original no formato BGR

    Saída:
    - resultado: imagem BGR com os nomes das figuras escritos 
    """


    resultado = img_bgr.copy()

    # Encontra os contornos da imgem em tons de cinza, onde a cor procurada é a preta
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # Para cada contorno, analiso sua bounding box isoladamente
    for contour in contours:
        # Pego apenas contornos de área relevante
        area_figura = cv2.contourArea(contour)
        if area_figura > 50:
            x0, y0, w, h = cv2.boundingRect(contour)
            roi = mask[y0:y0+h, x0:x0+w]

            ## Testamos a relação área_figura/área_bbox
            # Relação do retângulo/quadrado é 1
            if 0.9 < area_figura/(h*w) < 1.1:
                figura = "QUADRADO"
            elif 2.9/4 < area_figura/(h*w) < 3.5/4:
                figura = "CIRCULO"
            elif area_figura/(h*w) < 0.3:
                figura = "ESTRELA"
            else:
                figura = "???????"

            cv2.putText(resultado, figura, (x0+w//2, y0+h//2), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)

    return resultado

if __name__ == "__main__":
    bgr = cv2.imread('bitmap.png')
    resultado = encontra_figuras(bgr)

    cv2.imwrite("figura_q2_resultado.png", resultado)

    cv2.imshow('Original', bgr)
    cv2.imshow('Nomes', resultado)
    cv2.waitKey()
    cv2.destroyAllWindows()
