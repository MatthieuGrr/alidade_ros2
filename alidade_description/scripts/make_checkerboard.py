#!/usr/bin/env python3
"""Genere la mire de calibration en PDF, aux cotes exactes.

Damier de 9 x 7 cases, soit 8 x 6 COINS INTERIEURS -- c'est ce nombre-la que
prend cameracalibrator (--size 8x6). Cases de 25 mm, planche de 225 x 175 mm,
centree sur une A4 en paysage.

Damier classique et non ChArUco a dessein : une mire ChArUco porte des
marqueurs 4x4 que le detecteur du cube prendrait pour ses propres faces.

Le PDF est ecrit a la main plutot que par une bibliotheque : la cote imprimee
est ce qui fixe l'echelle de toutes les mesures ensuite, on ne veut aucune
transformation intermediaire.
"""
from __future__ import annotations

import sys

MM = 72.0 / 25.4          # points PostScript par millimetre
PAGE_W, PAGE_H = 297.0, 210.0     # A4 paysage, en mm
CASE = 25.0               # cote d'une case, en mm
COLS, ROWS = 9, 7         # cases ; les coins interieurs valent (8, 6)
MARGE_HAUT = 8.0


def contenu() -> str:
    board_w, board_h = COLS * CASE, ROWS * CASE
    x0 = (PAGE_W - board_w) / 2.0
    y0 = PAGE_H - MARGE_HAUT - board_h
    parties = ['0 g']
    for c in range(COLS):
        for r in range(ROWS):
            if (c + r) % 2:
                continue
            x = (x0 + c * CASE) * MM
            y = (y0 + (ROWS - 1 - r) * CASE) * MM
            parties.append(f'{x:.4f} {y:.4f} {CASE * MM:.4f} {CASE * MM:.4f} re f')

    # Traits de coupe aux coins de la planche : ils donnent une reference de
    # mesure sur toute la largeur, plus precise qu'une case isolee.
    parties.append('0.5 w')
    for x in (x0, x0 + board_w):
        parties.append(f'{x * MM:.4f} {(y0 - 4) * MM:.4f} m {x * MM:.4f} {(y0 - 1) * MM:.4f} l S')

    lignes = [
        (f'Damier de calibration Alidade  --  {COLS - 1}x{ROWS - 1} coins interieurs '
         f'(cameracalibrator --size {COLS - 1}x{ROWS - 1})'),
        (f'Case {CASE:.1f} mm  --  IMPRIMER A 100 %, sans « ajuster a la page »  --  '
         f'VERIFIER apres impression : {COLS} cases = {COLS * CASE:.0f},0 mm entre les deux traits'),
        'Coller sur un support rigide et parfaitement plan. Une feuille qui gondole fausse la distorsion.',
    ]
    for i, texte in enumerate(lignes):
        y = (y0 - 9.0 - i * 4.2) * MM
        parties.append(f'BT /F1 {8.5 if i else 9.5} Tf {x0 * MM:.4f} {y:.4f} Td '
                       f'({texte.replace("(", r"\(").replace(")", r"\)")}) Tj ET')
    return '\n'.join(parties)


def pdf() -> bytes:
    flux = contenu().encode('latin-1')
    objets = [
        b'<</Type /Catalog /Pages 2 0 R>>',
        b'<</Type /Pages /Kids [3 0 R] /Count 1>>',
        (f'<</Type /Page /Parent 2 0 R /MediaBox [0 0 {PAGE_W * MM:.4f} {PAGE_H * MM:.4f}] '
         f'/Contents 4 0 R /Resources <</Font <</F1 5 0 R>>>>>>').encode('latin-1'),
        b'<</Length ' + str(len(flux)).encode() + b'>>\nstream\n' + flux + b'\nendstream',
        b'<</Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding>>',
    ]
    out = bytearray(b'%PDF-1.4\n')
    offsets = []
    for i, obj in enumerate(objets, start=1):
        offsets.append(len(out))
        out += f'{i} 0 obj\n'.encode() + obj + b'\nendobj\n'
    xref = len(out)
    out += f'xref\n0 {len(objets) + 1}\n'.encode() + b'0000000000 65535 f \n'
    for off in offsets:
        out += f'{off:010d} 00000 n \n'.encode()
    out += (f'trailer\n<</Size {len(objets) + 1} /Root 1 0 R>>\nstartxref\n{xref}\n'
            '%%EOF\n').encode()
    return bytes(out)


if __name__ == '__main__':
    chemin = sys.argv[1] if len(sys.argv) > 1 else 'mire_calibration.pdf'
    open(chemin, 'wb').write(pdf())
    print(f'  {chemin}')
    print(f'  planche {COLS * CASE:.0f} x {ROWS * CASE:.0f} mm sur A4 paysage '
          f'({PAGE_W:.0f} x {PAGE_H:.0f} mm)')
    print(f'  {COLS}x{ROWS} cases de {CASE:.1f} mm  ->  '
          f'--size {COLS - 1}x{ROWS - 1} --square {CASE / 1000:.3f}')
