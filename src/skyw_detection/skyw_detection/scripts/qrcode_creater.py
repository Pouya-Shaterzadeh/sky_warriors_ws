#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import qrcode
import numpy as np
import yaml
import json

def create_qrcode(data, file_name, use_yaml=True):
    '''
    Creates QR code from mission data.
    use_yaml: If True, encode as YAML, else encode as JSON (competition format)
    '''
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=5,
        border=4)
    
    # Encode data based on format
    if use_yaml:
        qr_text = data if isinstance(data, str) else yaml.safe_dump(data, sort_keys=False)
    else:
        qr_text = data if isinstance(data, str) else json.dumps(data)
    
    qr.add_data(qr_text)
    qr.make(fit=True)
    
    img = qr.make_image(fill_color=(0, 0, 0), back_color=(255, 255, 255))
    opencv_img = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)
    
    while True:
        cv2.imshow('img', opencv_img)
        k = cv2.waitKey(1)
        if k != -1:
            break
    cv2.imwrite(file_name, opencv_img)
    print(f'Saved QR code to {file_name}')
    print(f'Content: {qr_text}')

if __name__ == '__main__':
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # Create simple mission QR code (YAML format)
    simple_config_path = os.path.join(project_root, 'config', 'qr-simple.yaml')
    simple_out_img = os.path.join(os.getcwd(), 'qr-simple.jpg')
    with open(simple_config_path, 'r', encoding='utf-8') as stream:
        simple_data = yaml.safe_load(stream)
    create_qrcode(simple_data, simple_out_img, use_yaml=True)
    
    # Create competition format QR code (JSON format)
    competition_config_path = os.path.join(project_root, 'config', 'qr.yaml')
    competition_out_img = os.path.join(os.getcwd(), 'qr-competition.jpg')
    with open(competition_config_path, 'r', encoding='utf-8') as stream:
        competition_data = yaml.safe_load(stream)
    create_qrcode(competition_data, competition_out_img, use_yaml=False)