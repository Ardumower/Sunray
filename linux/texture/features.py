# Feature extraction and NumPy MLP adapted from owl_wido/robot/texture-train-profile.py.
import os
import cv2
import numpy as np

def clamp_roi(roi, width, height):
    x = int(round(float(roi.get("x", 0))))
    y = int(round(float(roi.get("y", 0))))
    w = int(round(float(roi.get("width", width))))
    h = int(round(float(roi.get("height", height))))
    x1 = max(0, min(width - 1, x))
    y1 = max(0, min(height - 1, y))
    x2 = max(x1 + 1, min(width, x + w))
    y2 = max(y1 + 1, min(height, y + h))
    return x1, y1, x2, y2

def patch_positions(x1, y1, x2, y2, patch_size, include_partial=False):
    if include_partial:
        return [(x,y) for y in range(y1,y2,patch_size) for x in range(x1,x2,patch_size)]
    positions = []
    for y in range(y1, max(y1, y2 - patch_size) + 1, patch_size):
        for x in range(x1, max(x1, x2 - patch_size) + 1, patch_size):
            if x + patch_size <= x2 and y + patch_size <= y2:
                positions.append((x, y))
    return positions

def patch_vectors_from_map(image_shape, fmap, roi, patch_size, include_partial=False):
    height, width = image_shape[:2]
    x1, y1, x2, y2 = clamp_roi(roi, width, height)
    positions = patch_positions(x1, y1, x2, y2, patch_size, include_partial)
    if not positions:
        return np.empty((0, 1), dtype=np.float32)
    fh, fw, channels = fmap.shape
    vectors = []
    for x, y in positions:
        px2 = x + patch_size
        py2 = y + patch_size
        fx1 = max(0, min(fw - 1, int(np.floor((x / width) * fw))))
        fy1 = max(0, min(fh - 1, int(np.floor((y / height) * fh))))
        fx2 = max(fx1 + 1, min(fw, int(np.ceil((px2 / width) * fw))))
        fy2 = max(fy1 + 1, min(fh, int(np.ceil((py2 / height) * fh))))
        patch = fmap[fy1:fy2, fx1:fx2].reshape(-1, channels)
        vectors.append(np.concatenate([patch.mean(axis=0), patch.std(axis=0)]).astype(np.float32))
    return np.vstack(vectors)

def image_feature_map(rknn, rgb):
    input_size = int(os.environ.get("FULL_IMAGE_DISTILLED_RKNN_INPUT_SIZE", "512"))
    resized = cv2.resize(rgb, (input_size, input_size), interpolation=cv2.INTER_AREA)
    arr = resized.astype(np.float32) / 255.0
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    batch = np.ascontiguousarray(((arr - mean) / std)[None, ...], dtype=np.float32)
    outputs = rknn.inference(inputs=[batch])
    if not outputs:
        raise RuntimeError("Texture RKNN returned no outputs")
    fmap = np.asarray(outputs[0], dtype=np.float32).squeeze()
    if fmap.ndim == 3 and fmap.shape[0] in (160, 256, 320) and fmap.shape[1] <= 64:
        fmap = np.transpose(fmap, (1, 2, 0))
    elif fmap.ndim == 4:
        fmap = np.transpose(fmap[0], (1, 2, 0))
    if fmap.ndim != 3:
        raise RuntimeError(f"Unsupported texture RKNN output shape: {list(fmap.shape)}")
    return fmap / np.maximum(np.linalg.norm(fmap, axis=2, keepdims=True), 1e-6)

def mlp_predict_arrays(data, w1, b1, w2, b2):
    hidden = np.maximum(data.astype(np.float32) @ w1.astype(np.float32) + b1.astype(np.float32), 0.0)
    logits = np.clip(hidden @ w2.astype(np.float32).reshape(-1, 1) + float(np.array(b2, dtype=np.float32).reshape(-1)[0]), -40, 40)
    return (1.0 / (1.0 + np.exp(-logits))).reshape(-1).astype(np.float32)

def fit_mlp_classifier(normal_scaled, anomaly_scaled, seed=42, hidden=32, epochs=240, learning_rate=0.01, l2=0.01, validation_fraction=0.2, patience=25):
    x = np.vstack([normal_scaled, anomaly_scaled]).astype(np.float32)
    y = np.concatenate([
        np.zeros(len(normal_scaled), dtype=np.float32),
        np.ones(len(anomaly_scaled), dtype=np.float32),
    ])[:, None]
    rng = np.random.default_rng(seed)
    normal_indices = rng.permutation(np.where(y[:, 0] < 0.5)[0])
    anomaly_indices = rng.permutation(np.where(y[:, 0] > 0.5)[0])
    normal_val_count = max(1, int(len(normal_indices) * validation_fraction)) if len(normal_indices) > 5 else 0
    anomaly_val_count = max(1, int(len(anomaly_indices) * validation_fraction)) if len(anomaly_indices) > 5 else 0
    val_indices = np.concatenate([normal_indices[:normal_val_count], anomaly_indices[:anomaly_val_count]])
    train_indices = np.setdiff1d(np.arange(len(y)), val_indices, assume_unique=False)
    if len(train_indices) < 2 or len(val_indices) < 2:
        train_indices = np.arange(len(y))
        val_indices = np.arange(len(y))

    input_dim = x.shape[1]
    hidden = max(4, int(hidden))
    w1 = rng.normal(0, np.sqrt(2.0 / max(input_dim, 1)), size=(input_dim, hidden)).astype(np.float32)
    b1 = np.zeros((1, hidden), dtype=np.float32)
    w2 = rng.normal(0, np.sqrt(2.0 / hidden), size=(hidden, 1)).astype(np.float32)
    b2 = np.zeros((1, 1), dtype=np.float32)
    class_weights = np.where(y > 0.5, 0.5 / max(float(y.mean()), 1e-6), 0.5 / max(float(1.0 - y.mean()), 1e-6)).astype(np.float32)

    best = None
    best_loss = float("inf")
    stale = 0
    batch_size = min(256, max(32, len(train_indices)))
    for epoch in range(int(epochs)):
        shuffled = rng.permutation(train_indices)
        for start in range(0, len(shuffled), batch_size):
            batch = shuffled[start:start + batch_size]
            xb = x[batch]
            yb = y[batch]
            wb = class_weights[batch]
            z1 = xb @ w1 + b1
            h1 = np.maximum(z1, 0.0)
            logits = np.clip(h1 @ w2 + b2, -40, 40)
            probs = 1.0 / (1.0 + np.exp(-logits))
            delta2 = (probs - yb) * wb / len(batch)
            grad_w2 = h1.T @ delta2 + l2 * w2
            grad_b2 = delta2.sum(axis=0, keepdims=True)
            delta1 = (delta2 @ w2.T) * (z1 > 0)
            grad_w1 = xb.T @ delta1 + l2 * w1
            grad_b1 = delta1.sum(axis=0, keepdims=True)
            w2 -= learning_rate * grad_w2
            b2 -= learning_rate * grad_b2
            w1 -= learning_rate * grad_w1
            b1 -= learning_rate * grad_b1

        val_probs = mlp_predict_arrays(x[val_indices], w1, b1, w2, b2)[:, None]
        val_y = y[val_indices]
        val_w = class_weights[val_indices]
        eps = 1e-6
        val_loss = float((-(val_y * np.log(np.clip(val_probs, eps, 1.0)) + (1.0 - val_y) * np.log(np.clip(1.0 - val_probs, eps, 1.0))) * val_w).mean())
        if val_loss < best_loss - 1e-5:
            best_loss = val_loss
            best = (w1.copy(), b1.copy(), w2.copy(), b2.copy(), epoch + 1)
            stale = 0
        else:
            stale += 1
            if stale >= int(patience):
                break
    if best is not None:
        w1, b1, w2, b2, best_epoch = best
    else:
        best_epoch = int(epochs)
    return {
        "w1": w1.astype(np.float32),
        "b1": b1.reshape(-1).astype(np.float32),
        "w2": w2.reshape(-1).astype(np.float32),
        "b2": float(b2.reshape(-1)[0].item()),
        "epochs": int(best_epoch),
        "validationLoss": float(best_loss),
    }
