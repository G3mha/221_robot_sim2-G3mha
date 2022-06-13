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

    return resultado


if __name__ == "__main__":
    bgr = cv2.imread('bitmap.png')
    resultado = def encontra_figuras(img_bgr):
(bgr)

    cv2.imwrite("figura_q2_resultado.png", resultado)

    cv2.imshow('Original', bgr)
    cv2.imshow('Nomes', resultado)
    cv2.waitKey()
    cv2.destroyAllWindows()
