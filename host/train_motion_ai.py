"""
==============================================================================
MOTION AI - PC Training Script
==============================================================================

Train ML models on collected IMU data and export for Pico deployment.

USAGE:
    1. Collect data on robot using lab_motion_ai
    2. Export CSV from robot (press 'e')
    3. Save CSV to file: motion_data.csv
    4. Run this script: python train_motion_ai.py motion_data.csv

EDUCATIONAL LEVELS:
    Level 1: Visualize data - see what the features look like
    Level 2: Train KNN - simple, interpretable
    Level 3: Train Decision Tree - rule extraction
    Level 4: Train Neural Network - best accuracy, export to Pico
    Level 5: Feature importance - understand what matters

==============================================================================
"""

import sys
import csv
import math
import random
from collections import Counter

# Movement classes
CLASS_NAMES = ['STOPPED', 'FORWARD', 'BACKWARD', 'TURN_LEFT', 'TURN_RIGHT', 'PUSHED']


# ==============================================================================
# DATA LOADING
# ==============================================================================

def load_csv(filename):
    """Load training data from CSV."""
    X = []  # Features
    y = []  # Labels

    with open(filename, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)  # Skip header

        for row in reader:
            if len(row) < 2:
                continue
            features = [float(x) for x in row[:-1]]
            label = int(row[-1])
            X.append(features)
            y.append(label)

    print(f"Loaded {len(X)} samples from {filename}")
    print(f"Features per sample: {len(X[0]) if X else 0}")

    # Class distribution
    counts = Counter(y)
    print("\nClass distribution:")
    for label, count in sorted(counts.items()):
        pct = 100 * count / len(y)
        bar = '#' * int(pct / 2)
        print(f"  {CLASS_NAMES[label]:12s}: {count:4d} ({pct:5.1f}%) {bar}")

    return X, y


def split_data(X, y, test_ratio=0.2):
    """Split data into train/test sets."""
    indices = list(range(len(X)))
    random.shuffle(indices)

    split_idx = int(len(indices) * (1 - test_ratio))

    train_idx = indices[:split_idx]
    test_idx = indices[split_idx:]

    X_train = [X[i] for i in train_idx]
    y_train = [y[i] for i in train_idx]
    X_test = [X[i] for i in test_idx]
    y_test = [y[i] for i in test_idx]

    return X_train, X_test, y_train, y_test


# ==============================================================================
# LEVEL 1: DATA VISUALIZATION
# ==============================================================================

def visualize_data(X, y):
    """Visualize feature distributions (text-based)."""
    print("\n" + "=" * 60)
    print("FEATURE STATISTICS BY CLASS")
    print("=" * 60)

    n_features = len(X[0])
    feature_names = [f'f{i}' for i in range(n_features)]

    # Group by class
    by_class = {i: [] for i in range(6)}
    for features, label in zip(X, y):
        by_class[label].append(features)

    # Show key features
    key_features = [0, 1, 3, 4, 12, 13, 18, 19]  # ax_mean, ax_var, ay_mean, ay_var, gz_mean, gz_var, mag_mean, mag_var

    for feat_idx in key_features:
        print(f"\nFeature {feat_idx}:")
        for label in range(6):
            if not by_class[label]:
                continue
            values = [sample[feat_idx] for sample in by_class[label]]
            mean = sum(values) / len(values)
            min_v = min(values)
            max_v = max(values)
            print(f"  {CLASS_NAMES[label]:12s}: mean={mean:8.3f}  range=[{min_v:.3f}, {max_v:.3f}]")


# ==============================================================================
# LEVEL 2: K-NEAREST NEIGHBORS
# ==============================================================================

def train_knn(X_train, y_train, k=5):
    """Train KNN classifier."""
    print(f"\nTraining KNN (k={k})...")

    # KNN just stores the training data
    model = {
        'type': 'knn',
        'k': k,
        'X': X_train,
        'y': y_train
    }

    print(f"KNN model: {len(X_train)} training samples")
    return model


def predict_knn(model, x):
    """Predict using KNN."""
    # Calculate distances
    distances = []
    for i, train_x in enumerate(model['X']):
        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(x, train_x)))
        distances.append((dist, model['y'][i]))

    # Sort and get k nearest
    distances.sort(key=lambda d: d[0])
    k_nearest = distances[:model['k']]

    # Vote
    votes = Counter(label for _, label in k_nearest)
    return votes.most_common(1)[0][0]


# ==============================================================================
# LEVEL 3: DECISION TREE (SIMPLE)
# ==============================================================================

def train_decision_tree(X_train, y_train, max_depth=5):
    """Train simple decision tree."""
    print(f"\nTraining Decision Tree (max_depth={max_depth})...")

    def gini(labels):
        """Gini impurity."""
        if not labels:
            return 0
        counts = Counter(labels)
        n = len(labels)
        return 1 - sum((c / n) ** 2 for c in counts.values())

    def find_best_split(X, y, features):
        """Find best feature and threshold to split on."""
        best_gain = -1
        best_feature = None
        best_threshold = None

        current_gini = gini(y)

        for feat_idx in features:
            values = sorted(set(x[feat_idx] for x in X))
            thresholds = [(values[i] + values[i+1]) / 2 for i in range(len(values)-1)]

            for thresh in thresholds[:10]:  # Limit thresholds checked
                left_y = [y[i] for i, x in enumerate(X) if x[feat_idx] <= thresh]
                right_y = [y[i] for i, x in enumerate(X) if x[feat_idx] > thresh]

                if not left_y or not right_y:
                    continue

                # Information gain
                p_left = len(left_y) / len(y)
                p_right = len(right_y) / len(y)
                gain = current_gini - p_left * gini(left_y) - p_right * gini(right_y)

                if gain > best_gain:
                    best_gain = gain
                    best_feature = feat_idx
                    best_threshold = thresh

        return best_feature, best_threshold, best_gain

    def build_tree(X, y, depth):
        """Recursively build tree."""
        # Stopping conditions
        if depth >= max_depth or len(set(y)) == 1 or len(X) < 5:
            return {'leaf': True, 'class': Counter(y).most_common(1)[0][0]}

        # Find best split
        features = list(range(len(X[0])))
        feat_idx, threshold, gain = find_best_split(X, y, features)

        if feat_idx is None or gain < 0.01:
            return {'leaf': True, 'class': Counter(y).most_common(1)[0][0]}

        # Split data
        left_X = [X[i] for i in range(len(X)) if X[i][feat_idx] <= threshold]
        left_y = [y[i] for i in range(len(X)) if X[i][feat_idx] <= threshold]
        right_X = [X[i] for i in range(len(X)) if X[i][feat_idx] > threshold]
        right_y = [y[i] for i in range(len(X)) if X[i][feat_idx] > threshold]

        return {
            'leaf': False,
            'feature': feat_idx,
            'threshold': threshold,
            'left': build_tree(left_X, left_y, depth + 1),
            'right': build_tree(right_X, right_y, depth + 1)
        }

    tree = build_tree(X_train, y_train, 0)

    # Print tree rules
    def print_tree(node, indent=""):
        if node['leaf']:
            print(f"{indent}→ {CLASS_NAMES[node['class']]}")
        else:
            print(f"{indent}if feature[{node['feature']}] <= {node['threshold']:.3f}:")
            print_tree(node['left'], indent + "  ")
            print(f"{indent}else:")
            print_tree(node['right'], indent + "  ")

    print("\nDecision tree rules:")
    print_tree(tree)

    return {'type': 'tree', 'tree': tree}


def predict_tree(model, x):
    """Predict using decision tree."""
    node = model['tree']
    while not node['leaf']:
        if x[node['feature']] <= node['threshold']:
            node = node['left']
        else:
            node = node['right']
    return node['class']


# ==============================================================================
# LEVEL 4: NEURAL NETWORK
# ==============================================================================

class NeuralNetwork:
    """Simple neural network with backpropagation training."""

    def __init__(self, input_size, hidden_size, output_size):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size

        # Xavier initialization
        scale1 = math.sqrt(2.0 / input_size)
        scale2 = math.sqrt(2.0 / hidden_size)

        self.W1 = [[random.gauss(0, scale1) for _ in range(input_size)]
                   for _ in range(hidden_size)]
        self.b1 = [0.0] * hidden_size
        self.W2 = [[random.gauss(0, scale2) for _ in range(hidden_size)]
                   for _ in range(output_size)]
        self.b2 = [0.0] * output_size

    def relu(self, x):
        return max(0, x)

    def relu_deriv(self, x):
        return 1 if x > 0 else 0

    def softmax(self, x):
        max_x = max(x)
        exp_x = [math.exp(xi - max_x) for xi in x]
        sum_exp = sum(exp_x)
        return [e / sum_exp for e in exp_x]

    def forward(self, x):
        """Forward pass, returns all intermediate values for backprop."""
        # Hidden layer
        z1 = [self.b1[i] + sum(self.W1[i][j] * x[j] for j in range(self.input_size))
              for i in range(self.hidden_size)]
        h = [self.relu(z) for z in z1]

        # Output layer
        z2 = [self.b2[i] + sum(self.W2[i][j] * h[j] for j in range(self.hidden_size))
              for i in range(self.output_size)]
        out = self.softmax(z2)

        return {'x': x, 'z1': z1, 'h': h, 'z2': z2, 'out': out}

    def backward(self, cache, y_true, lr):
        """Backpropagation."""
        x, h, out = cache['x'], cache['h'], cache['out']
        z1 = cache['z1']

        # Output layer gradient
        d_out = out[:]
        d_out[y_true] -= 1  # Cross-entropy gradient

        # Hidden layer gradient
        d_h = [sum(self.W2[i][j] * d_out[i] for i in range(self.output_size))
               for j in range(self.hidden_size)]
        d_z1 = [d_h[i] * self.relu_deriv(z1[i]) for i in range(self.hidden_size)]

        # Update weights
        for i in range(self.output_size):
            for j in range(self.hidden_size):
                self.W2[i][j] -= lr * d_out[i] * h[j]
            self.b2[i] -= lr * d_out[i]

        for i in range(self.hidden_size):
            for j in range(self.input_size):
                self.W1[i][j] -= lr * d_z1[i] * x[j]
            self.b1[i] -= lr * d_z1[i]

    def predict(self, x):
        """Predict class."""
        out = self.forward(x)['out']
        return out.index(max(out))


def train_neural_network(X_train, y_train, hidden_size=10, epochs=100, lr=0.01):
    """Train neural network."""
    print(f"\nTraining Neural Network (hidden={hidden_size}, epochs={epochs})...")

    input_size = len(X_train[0])
    output_size = 6

    nn = NeuralNetwork(input_size, hidden_size, output_size)

    # Normalize features
    means = [sum(X[i] for X in X_train) / len(X_train) for i in range(input_size)]
    stds = [math.sqrt(sum((X[i] - means[i])**2 for X in X_train) / len(X_train)) or 1
            for i in range(input_size)]

    X_norm = [[(x[i] - means[i]) / stds[i] for i in range(input_size)] for x in X_train]

    # Training loop
    for epoch in range(epochs):
        # Shuffle
        indices = list(range(len(X_norm)))
        random.shuffle(indices)

        total_loss = 0
        correct = 0

        for idx in indices:
            x = X_norm[idx]
            y = y_train[idx]

            cache = nn.forward(x)
            pred = cache['out'].index(max(cache['out']))

            # Cross-entropy loss
            total_loss -= math.log(cache['out'][y] + 1e-10)
            if pred == y:
                correct += 1

            nn.backward(cache, y, lr)

        if (epoch + 1) % 20 == 0:
            acc = 100 * correct / len(X_norm)
            print(f"  Epoch {epoch+1:3d}: loss={total_loss:.3f}, accuracy={acc:.1f}%")

    return {
        'type': 'nn',
        'nn': nn,
        'means': means,
        'stds': stds
    }


def predict_nn(model, x):
    """Predict using neural network."""
    # Normalize
    x_norm = [(x[i] - model['means'][i]) / model['stds'][i] for i in range(len(x))]
    return model['nn'].predict(x_norm)


# ==============================================================================
# EVALUATION
# ==============================================================================

def evaluate(model, X_test, y_test):
    """Evaluate model accuracy."""
    if model['type'] == 'knn':
        predict_fn = lambda x: predict_knn(model, x)
    elif model['type'] == 'tree':
        predict_fn = lambda x: predict_tree(model, x)
    elif model['type'] == 'nn':
        predict_fn = lambda x: predict_nn(model, x)

    correct = 0
    confusion = [[0] * 6 for _ in range(6)]

    for x, y in zip(X_test, y_test):
        pred = predict_fn(x)
        if pred == y:
            correct += 1
        confusion[y][pred] += 1

    accuracy = 100 * correct / len(y_test)

    print(f"\nAccuracy: {accuracy:.1f}% ({correct}/{len(y_test)})")

    print("\nConfusion Matrix:")
    print("         " + "  ".join(f"{name[:6]:>6s}" for name in CLASS_NAMES))
    for i, row in enumerate(confusion):
        print(f"{CLASS_NAMES[i]:8s} " + "  ".join(f"{v:>6d}" for v in row))

    return accuracy


# ==============================================================================
# EXPORT FOR PICO
# ==============================================================================

def export_nn_for_pico(model, filename='nn_weights.py'):
    """Export neural network weights as Python code for Pico."""
    nn = model['nn']

    code = '''"""
Neural Network weights for Pico deployment.
Generated by train_motion_ai.py

Usage:
    from nn_weights import W1, b1, W2, b2, MEANS, STDS
    # Then use with SimpleNeuralNetwork.load_model()
"""

'''
    # Weights
    code += f"INPUT_SIZE = {nn.input_size}\n"
    code += f"HIDDEN_SIZE = {nn.hidden_size}\n"
    code += f"OUTPUT_SIZE = {nn.output_size}\n\n"

    # Feature normalization
    code += "MEANS = [\n"
    for m in model['means']:
        code += f"    {m:.6f},\n"
    code += "]\n\n"

    code += "STDS = [\n"
    for s in model['stds']:
        code += f"    {s:.6f},\n"
    code += "]\n\n"

    # W1
    code += "W1 = [\n"
    for row in nn.W1:
        code += "    [" + ", ".join(f"{w:.6f}" for w in row) + "],\n"
    code += "]\n\n"

    # b1
    code += "b1 = [" + ", ".join(f"{b:.6f}" for b in nn.b1) + "]\n\n"

    # W2
    code += "W2 = [\n"
    for row in nn.W2:
        code += "    [" + ", ".join(f"{w:.6f}" for w in row) + "],\n"
    code += "]\n\n"

    # b2
    code += "b2 = [" + ", ".join(f"{b:.6f}" for b in nn.b2) + "]\n"

    with open(filename, 'w') as f:
        f.write(code)

    print(f"\nExported weights to {filename}")
    print("Copy this file to Pico and import in motion_ai.py")


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    if len(sys.argv) < 2:
        print("Usage: python train_motion_ai.py <data.csv>")
        print("\nTo generate test data, run lab_motion_ai on robot first.")
        sys.exit(1)

    filename = sys.argv[1]

    # Load data
    X, y = load_csv(filename)

    if len(X) < 20:
        print("Need more training data! Collect at least 20 samples per class.")
        sys.exit(1)

    # Split
    X_train, X_test, y_train, y_test = split_data(X, y, test_ratio=0.2)
    print(f"\nSplit: {len(X_train)} train, {len(X_test)} test")

    # Level 1: Visualize
    visualize_data(X, y)

    # Level 2: KNN
    print("\n" + "=" * 60)
    print("LEVEL 2: K-NEAREST NEIGHBORS")
    print("=" * 60)
    knn_model = train_knn(X_train, y_train, k=5)
    evaluate(knn_model, X_test, y_test)

    # Level 3: Decision Tree
    print("\n" + "=" * 60)
    print("LEVEL 3: DECISION TREE")
    print("=" * 60)
    tree_model = train_decision_tree(X_train, y_train, max_depth=4)
    evaluate(tree_model, X_test, y_test)

    # Level 4: Neural Network
    print("\n" + "=" * 60)
    print("LEVEL 4: NEURAL NETWORK")
    print("=" * 60)
    nn_model = train_neural_network(X_train, y_train, hidden_size=10, epochs=100, lr=0.01)
    evaluate(nn_model, X_test, y_test)

    # Export for Pico
    export_nn_for_pico(nn_model, 'nn_weights.py')

    print("\n" + "=" * 60)
    print("DONE! Next steps:")
    print("=" * 60)
    print("1. Copy nn_weights.py to Pico")
    print("2. Modify motion_ai.py to load these weights")
    print("3. Run in test mode to see predictions")


if __name__ == '__main__':
    main()
