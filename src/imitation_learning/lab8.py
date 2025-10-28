#!/usr/bin/env python3

import os
import zipfile
import requests
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow import keras
from keras import layers

# --- CONFIGURATION ---
DATA_URL = 'https://archive.ics.uci.edu/static/public/846/accelerometer.zip'
ZIP_NAME = 'accelerometer.zip'
EXTRACT_DIR = 'accelerometer_data'
CSV_NAME = 'accelerometer (1).csv'

# --- UTILITIES ---
def download_and_unzip():
    if not os.path.exists(ZIP_NAME):
        print(f"Downloading {ZIP_NAME}...")
        r = requests.get(DATA_URL)
        r.raise_for_status()
        with open(ZIP_NAME, 'wb') as f:
            f.write(r.content)
    if not os.path.isdir(EXTRACT_DIR):
        print(f"Extracting {ZIP_NAME}...")
        with zipfile.ZipFile(ZIP_NAME, 'r') as z:
            z.extractall(EXTRACT_DIR)

def load_dataframe():
    path = os.path.join(EXTRACT_DIR, CSV_NAME)
    return pd.read_csv(path)

def plot_history(history, title_prefix, metric, val_metric):
    plt.plot(history.history[metric],    label='train')
    plt.plot(history.history[val_metric],label='val')
    plt.title(f'{title_prefix} {metric}')
    plt.xlabel('Epoch'); plt.ylabel(metric)
    plt.legend(loc='best')
    plt.show()

# --- CLASSIFICATION TASK ---
def run_classification(df):
    print("\n=== Classification Task ===")
    # 1. Prepare inputs X and one-hot targets y_cat
    X = df[['x','y','z']].values
    y = df['wconfid'].values.astype(int) - 1   # shift to 0,1,2
    num_classes = len(np.unique(y))
    y_cat = keras.utils.to_categorical(y, num_classes)

    # 2. Split: 70% train, 15% val, 15% test
    X_train, X_tmp, y_train, y_tmp = train_test_split(
        X, y_cat, test_size=0.30, random_state=42, stratify=y)
    X_val, X_test, y_val, y_test = train_test_split(
        X_tmp, y_tmp, test_size=0.50, random_state=42,
        stratify=np.argmax(y_tmp, axis=1))

    # 3. Scale inputs
    scaler = StandardScaler().fit(X_train)
    X_train = scaler.transform(X_train)
    X_val   = scaler.transform(X_val)
    X_test  = scaler.transform(X_test)

    # 4. Build & train 3 architectures
    archs = {
        'A': [16, 16],
        'B': [32, 16, 8],
        'C': [64]
    }
    for name, layers_cfg in archs.items():
        print(f"\n-- Model {name}, layers {layers_cfg}")
        inp = keras.Input(shape=(3,))
        x = inp
        for units in layers_cfg:
            x = layers.Dense(units, activation='relu')(x)
        out = layers.Dense(num_classes, activation='softmax')(x)
        model = keras.Model(inputs=inp, outputs=out)
        model.compile(
            loss='categorical_crossentropy',
            optimizer='adam',
            metrics=['accuracy'])
        history = model.fit(
            X_train, y_train,
            validation_data=(X_val, y_val),
            epochs=20, batch_size=128, verbose=2)
        loss, acc = model.evaluate(X_test, y_test, verbose=0)
        print(f"Model {name} Test Accuracy: {acc:.4f}")
        plot_history(history, f'Clf {name}', 'accuracy', 'val_accuracy')
        plot_history(history, f'Clf {name}', 'loss',     'val_loss')

# --- REGRESSION TASK ---
def run_regression(df):
    print("\n=== Regression Task ===")
    # 1. Prepare inputs X and continuous target y_reg (scaled to [0,1])
    X = df[['x','y','z']].values
    y_reg = df['pctid'].values.astype(np.float32) / 100.0

    # 2. Split: 70% train, 15% val, 15% test
    X_train, X_tmp, y_train, y_tmp = train_test_split(
        X, y_reg, test_size=0.30, random_state=42)
    X_val, X_test, y_val, y_test = train_test_split(
        X_tmp, y_tmp, test_size=0.50, random_state=42)

    # 3. Scale inputs
    scaler = StandardScaler().fit(X_train)
    X_train = scaler.transform(X_train)
    X_val   = scaler.transform(X_val)
    X_test  = scaler.transform(X_test)

    # 4. Build & train 3 architectures
    archs = {
        'R1': [16],
        'R2': [32, 16],
        'R3': [16, 16, 16]
    }
    for name, layers_cfg in archs.items():
        print(f"\n-- Model {name}, layers {layers_cfg}")
        inp = keras.Input(shape=(3,))
        x = inp
        for units in layers_cfg:
            x = layers.Dense(units, activation='relu')(x)
        out = layers.Dense(1, activation=None)(x)
        model = keras.Model(inputs=inp, outputs=out)
        model.compile(
            loss='mse',
            optimizer='adam',
            metrics=['mae'])
        history = model.fit(
            X_train, y_train,
            validation_data=(X_val, y_val),
            epochs=20, batch_size=128, verbose=2)
        loss, mae = model.evaluate(X_test, y_test, verbose=0)
        print(f"Model {name} Test MSE: {loss:.4f}, MAE: {mae:.4f}")
        plot_history(history, f'Reg {name}', 'mae',     'val_mae')
        plot_history(history, f'Reg {name}', 'loss',    'val_loss')

# --- MAIN EXECUTION ---
def main():
    download_and_unzip()
    df = load_dataframe()
    run_classification(df)
    run_regression(df)

if __name__ == '__main__':
    main()
