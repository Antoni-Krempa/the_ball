"""Wlasny filtr komplementarny orientacji: gyro + korekcja acc.

Wymagania: numpy, pandas, matplotlib.
Uruchomienie: python filtr_wlasny_acc_gyro.py
Pliki wejsciowe i wynikowe sa odczytywane/zapisywane obok skryptu.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def euler_zyx_to_quat(roll, pitch, yaw):
    """Kwaternion [qw, qx, qy, qz], katy w radianach."""
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    q = np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])
    return q / np.linalg.norm(q)


def quat_to_euler_zyx(q):
    qw, qx, qy, qz = q
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    pitch = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1))
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
    return np.array([roll, pitch, yaw])


def wrap_pi(x):
    return np.arctan2(np.sin(x), np.cos(x))


def wrap_deg(x):
    return (x + 180) % 360 - 180


def main():
    folder = Path(__file__).resolve().parent
    dane = pd.read_csv(folder / "train.csv")
    if dane.empty:
        raise ValueError("Plik train.csv nie zawiera probek.")
    t = dane["Time"].to_numpy(dtype=float)
    gyro_mdps = dane[["GyroX", "GyroY", "GyroZ"]].to_numpy(dtype=float)
    acc = dane[["AccX", "AccY", "AccZ"]].to_numpy(dtype=float)
    n = len(dane)

    # Pierwsze 5 s: czujnik nieruchomy i ustawiony poziomo, Z do gory.
    probki_biasu = t < t[0] + 5
    if not np.any(probki_biasu):
        raise ValueError("Brak probek do kalibracji biasu gyro.")
    bias_mdps = np.nanmean(gyro_mdps[probki_biasu], axis=0)
    omega = (gyro_mdps - bias_mdps) * (np.pi / 180000)
    print("Usuniety bias [mdps]: X=%.2f, Y=%.2f, Z=%.2f" % tuple(bias_mdps))

    mean_acc0 = np.nanmean(acc[probki_biasu], axis=0)
    g_ref0 = np.linalg.norm(mean_acc0)
    acc_bias = mean_acc0 - np.array([0, 0, g_ref0])
    acc_cal = acc - acc_bias

    tau = 0.5  # [s] mniejsze tau = mocniejsza korekta
    norma_acc = np.linalg.norm(acc_cal, axis=1)
    g_ref = np.nanmedian(norma_acc[probki_biasu])
    if not np.isfinite(g_ref) or g_ref < 1e-9:
        raise ValueError("Nie udalo sie wyznaczyc normy przyspieszenia w postoju.")
    tolerancja_normy = 0.30 * g_ref
    print(f"Referencyjna norma acc: {g_ref:.5g} (jednostka jak w CSV)")
    print("Usuniety staly offset acc: X=%.5g, Y=%.5g, Z=%.5g" % tuple(acc_bias))

    q = np.zeros((n, 4))
    q[0] = [1, 0, 0, 0]
    euler = np.zeros((n, 3))
    euler[0] = quat_to_euler_zyx(q[0])

    for k in range(1, n):
        dt = t[k] - t[k - 1]
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError(f"Niepoprawny krok czasu przy probce {k + 1}.")

        # Predykcja z gyro, identyczna konwencja jak w skrypcie MATLAB.
        wx, wy, wz = omega[k - 1]
        omega_matrix = np.array([
            [0, -wx, -wy, -wz],
            [wx, 0, wz, -wy],
            [wy, -wz, 0, wx],
            [wz, wy, -wx, 0],
        ])
        q_pred = (np.eye(4) + 0.5 * omega_matrix * dt) @ q[k - 1]
        q_pred /= np.linalg.norm(q_pred)
        roll_gyro, pitch_gyro, yaw_gyro = quat_to_euler_zyx(q_pred)

        # Pomiar przechylenia z acc przy zalozeniu dominacji grawitacji.
        ax, ay, az = acc_cal[k]
        roll_acc = np.arctan2(ay, az)
        pitch_acc = np.arctan2(-ax, np.hypot(ay, az))
        confidence = np.clip(1 - abs(norma_acc[k] - g_ref) / tolerancja_normy, 0, 1)
        beta = dt / (tau + dt) * confidence

        roll = roll_gyro + beta * wrap_pi(roll_acc - roll_gyro)
        pitch = pitch_gyro + beta * wrap_pi(pitch_acc - pitch_gyro)
        # Acc nie wyznacza yaw: pozostaje wartosc z predykcji gyro.
        q[k] = euler_zyx_to_quat(roll, pitch, yaw_gyro)
        euler[k] = quat_to_euler_zyx(q[k])

    euler = np.rad2deg(euler)
    referencja = dane[["roll", "pitch", "yaw"]].to_numpy(dtype=float)
    blad = wrap_deg(euler - referencja)
    rmse = np.sqrt(np.nanmean(blad**2, axis=0))
    for nazwa, wartosc in zip(("roll", "pitch", "yaw"), rmse):
        print(f"RMSE {nazwa}: {wartosc:.3f} deg")

    wynik = pd.DataFrame({"Time": t})
    for j, nazwa in enumerate(("qw", "qx", "qy", "qz")):
        wynik[nazwa] = q[:, j]
    for j, nazwa in enumerate(("roll", "pitch", "yaw")):
        wynik[f"{nazwa}_obliczony"] = euler[:, j]
    for j, nazwa in enumerate(("roll", "pitch", "yaw")):
        wynik[f"{nazwa}_referencyjny"] = referencja[:, j]
    # Osobny wynik, aby nie nadpisywac rezultatu skryptu MATLAB.
    sciezka_wyniku = folder / "wynik_filtru_wlasnego_python.csv"
    wynik.to_csv(sciezka_wyniku, index=False)
    print(f"Zapisano: {sciezka_wyniku}")

    fig, axes = plt.subplots(3, 1, sharex=True, num="Wlasny filtr: porownanie z referencja")
    for j, (nazwa, kolor) in enumerate(zip(("Roll", "Pitch", "Yaw"), ("r", "g", "b"))):
        axes[j].plot(t, referencja[:, j], "k", linewidth=1, label="Referencja")
        axes[j].plot(t, euler[:, j], kolor, linewidth=1, label="Filtr")
        axes[j].set_ylabel(f"{nazwa} [deg]")
        axes[j].grid(True)
        axes[j].legend()
    axes[-1].set_xlabel("Czas [s]")
    fig.suptitle("Wlasny filtr komplementarny: gyro + korekcja acc")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
