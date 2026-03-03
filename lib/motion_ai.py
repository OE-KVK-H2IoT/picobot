"""
==============================================================================
MOTION AI - Machine Learning Movement Detection for PicoBot
==============================================================================

Educational progression from basic statistics to AI:

LEVEL 1: Statistics
    - Mean, variance, thresholds
    - Simple rule-based detection

LEVEL 2: Feature Engineering
    - Extract meaningful features from raw sensor data
    - Time-domain and frequency-domain features

LEVEL 3: Classical ML
    - K-Nearest Neighbors (KNN)
    - Decision Tree (simple version)
    - Can run on microcontroller!

LEVEL 4: Neural Network
    - Simple MLP (Multi-Layer Perceptron)
    - Train on PC, deploy weights to Pico

MOVEMENT CLASSES:
    0 = STOPPED     - Robot not moving
    1 = FORWARD     - Moving forward
    2 = BACKWARD    - Moving backward
    3 = TURN_LEFT   - Rotating left
    4 = TURN_RIGHT  - Rotating right
    5 = PUSHED      - External force detected

==============================================================================
"""

import math
import time

# Movement class labels
STOPPED = 0
FORWARD = 1
BACKWARD = 2
TURN_LEFT = 3
TURN_RIGHT = 4
PUSHED = 5

CLASS_NAMES = ['STOPPED', 'FORWARD', 'BACKWARD', 'TURN_LEFT', 'TURN_RIGHT', 'PUSHED']


# ==============================================================================
# LEVEL 1: STATISTICAL FEATURES
# ==============================================================================

class FeatureExtractor:
    """
    Extract statistical features from sensor data windows.

    Features extracted (per axis + combined):
    - mean: Average value
    - var: Variance (spread of values)
    - std: Standard deviation
    - min/max: Range
    - rms: Root mean square
    - zero_crossings: How often signal crosses mean
    """

    def __init__(self, window_size=50):
        self.window_size = window_size
        self.reset()

    def reset(self):
        """Clear all buffers."""
        self.ax_buf = []
        self.ay_buf = []
        self.az_buf = []
        self.gx_buf = []
        self.gy_buf = []
        self.gz_buf = []

    def add_sample(self, ax, ay, az, gx, gy, gz):
        """Add a new sensor sample."""
        self.ax_buf.append(ax)
        self.ay_buf.append(ay)
        self.az_buf.append(az)
        self.gx_buf.append(gx)
        self.gy_buf.append(gy)
        self.gz_buf.append(gz)

        # Keep window size
        if len(self.ax_buf) > self.window_size:
            self.ax_buf.pop(0)
            self.ay_buf.pop(0)
            self.az_buf.pop(0)
            self.gx_buf.pop(0)
            self.gy_buf.pop(0)
            self.gz_buf.pop(0)

    def is_ready(self):
        """Check if we have enough samples."""
        return len(self.ax_buf) >= self.window_size // 2

    @staticmethod
    def _mean(data):
        return sum(data) / len(data) if data else 0

    @staticmethod
    def _variance(data):
        if len(data) < 2:
            return 0
        m = sum(data) / len(data)
        return sum((x - m) ** 2 for x in data) / len(data)

    @staticmethod
    def _std(data):
        return math.sqrt(FeatureExtractor._variance(data))

    @staticmethod
    def _rms(data):
        if not data:
            return 0
        return math.sqrt(sum(x * x for x in data) / len(data))

    @staticmethod
    def _zero_crossings(data):
        if len(data) < 2:
            return 0
        mean = sum(data) / len(data)
        crossings = 0
        above = data[0] > mean
        for x in data[1:]:
            now_above = x > mean
            if now_above != above:
                crossings += 1
                above = now_above
        return crossings

    def extract_features(self):
        """
        Extract feature vector from current window.

        Returns:
            List of features (20 values)
        """
        if not self.is_ready():
            return None

        features = []

        # Accelerometer features
        for buf in [self.ax_buf, self.ay_buf, self.az_buf]:
            features.append(self._mean(buf))
            features.append(self._variance(buf) * 100)  # Scale up
            features.append(self._zero_crossings(buf))

        # Gyroscope features
        for buf in [self.gx_buf, self.gy_buf, self.gz_buf]:
            features.append(self._mean(buf))
            features.append(self._variance(buf))

        # Combined features
        accel_mag = [math.sqrt(x*x + y*y + z*z)
                     for x, y, z in zip(self.ax_buf, self.ay_buf, self.az_buf)]
        features.append(self._mean(accel_mag))
        features.append(self._variance(accel_mag) * 100)

        return features

    def get_feature_names(self):
        """Get names of all features."""
        names = []
        for axis in ['ax', 'ay', 'az']:
            names.extend([f'{axis}_mean', f'{axis}_var', f'{axis}_zc'])
        for axis in ['gx', 'gy', 'gz']:
            names.extend([f'{axis}_mean', f'{axis}_var'])
        names.extend(['accel_mag_mean', 'accel_mag_var'])
        return names


# ==============================================================================
# LEVEL 2: RULE-BASED CLASSIFIER
# ==============================================================================

class RuleBasedClassifier:
    """
    Simple rule-based movement classifier using thresholds.

    This is the baseline - easy to understand, no training needed.
    Students can tune thresholds manually.
    """

    def __init__(self):
        # Thresholds (tune these!)
        self.gyro_turn_threshold = 10.0      # deg/s for turn detection
        self.accel_move_threshold = 0.08     # g for movement detection
        self.vibration_threshold = 2.0       # variance for wheel motion
        self.stationary_var_threshold = 0.5  # low variance = stopped

    def predict(self, features):
        """
        Predict movement class from features.

        Args:
            features: List from FeatureExtractor.extract_features()

        Returns:
            Predicted class (0-5)
        """
        if features is None or len(features) < 20:
            return STOPPED

        # Extract relevant features
        ax_mean = features[0]
        ax_var = features[1]
        ay_mean = features[3]
        gz_mean = features[12]
        gz_var = features[13]
        accel_mag_var = features[19]

        # Rule 1: Check rotation (turning)
        if abs(gz_mean) > self.gyro_turn_threshold:
            if gz_mean > 0:
                return TURN_LEFT
            else:
                return TURN_RIGHT

        # Rule 2: Check if stationary (low variance on all)
        if accel_mag_var < self.stationary_var_threshold and gz_var < 1.0:
            return STOPPED

        # Rule 3: Check for sudden push (high variance spike)
        if accel_mag_var > self.vibration_threshold * 3:
            return PUSHED

        # Rule 4: Check linear motion from acceleration
        if accel_mag_var > self.vibration_threshold:
            # Use mean acceleration to determine direction
            # (This is approximate - depends on IMU orientation)
            if ax_mean > self.accel_move_threshold:
                return FORWARD
            elif ax_mean < -self.accel_move_threshold:
                return BACKWARD
            else:
                return FORWARD  # Default to forward if moving but unclear

        return STOPPED

    def predict_proba(self, features):
        """Return confidence scores (simplified)."""
        pred = self.predict(features)
        proba = [0.1] * 6  # Small base probability
        proba[pred] = 0.8  # High confidence for prediction
        return proba


# ==============================================================================
# LEVEL 3: K-NEAREST NEIGHBORS (KNN)
# ==============================================================================

class KNNClassifier:
    """
    K-Nearest Neighbors classifier - simple ML that works on microcontrollers.

    How it works:
    1. Store training examples with their labels
    2. For new sample, find K closest training examples
    3. Vote: majority class among K neighbors = prediction

    Pros: Simple, no training time, interpretable
    Cons: Slow prediction if many training samples, needs good features
    """

    def __init__(self, k=5, max_samples=200):
        self.k = k
        self.max_samples = max_samples
        self.training_data = []  # List of (features, label)

    def add_training_sample(self, features, label):
        """Add a training example."""
        if features is None:
            return
        self.training_data.append((features[:], label))  # Copy features

        # Keep max samples (remove oldest if full)
        if len(self.training_data) > self.max_samples:
            self.training_data.pop(0)

    def _distance(self, f1, f2):
        """Euclidean distance between feature vectors."""
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(f1, f2)))

    def predict(self, features):
        """Predict class for new sample."""
        if not self.training_data or features is None:
            return STOPPED

        # Calculate distances to all training samples
        distances = []
        for train_features, label in self.training_data:
            dist = self._distance(features, train_features)
            distances.append((dist, label))

        # Sort by distance
        distances.sort(key=lambda x: x[0])

        # Get K nearest neighbors
        k_nearest = distances[:self.k]

        # Vote
        votes = [0] * 6
        for dist, label in k_nearest:
            votes[label] += 1

        # Return class with most votes
        return votes.index(max(votes))

    def predict_proba(self, features):
        """Return probability estimates based on neighbor votes."""
        if not self.training_data or features is None:
            return [1/6] * 6

        distances = []
        for train_features, label in self.training_data:
            dist = self._distance(features, train_features)
            distances.append((dist, label))

        distances.sort(key=lambda x: x[0])
        k_nearest = distances[:self.k]

        votes = [0] * 6
        for dist, label in k_nearest:
            votes[label] += 1

        total = sum(votes)
        return [v / total for v in votes] if total > 0 else [1/6] * 6

    def get_training_size(self):
        """Return number of training samples."""
        return len(self.training_data)

    def save_model(self):
        """Export model as string (for saving to file)."""
        lines = [f"KNN,{self.k},{len(self.training_data)}"]
        for features, label in self.training_data:
            feat_str = ','.join(f"{f:.6f}" for f in features)
            lines.append(f"{label},{feat_str}")
        return '\n'.join(lines)

    def load_model(self, data):
        """Load model from string."""
        lines = data.strip().split('\n')
        header = lines[0].split(',')
        self.k = int(header[1])
        self.training_data = []

        for line in lines[1:]:
            parts = line.split(',')
            label = int(parts[0])
            features = [float(x) for x in parts[1:]]
            self.training_data.append((features, label))


# ==============================================================================
# LEVEL 4: SIMPLE NEURAL NETWORK
# ==============================================================================

class SimpleNeuralNetwork:
    """
    Simple 2-layer neural network (MLP) for movement classification.

    Architecture:
        Input (20 features) -> Hidden (10 neurons) -> Output (6 classes)

    Can run on Pico! Train on PC, export weights, load on Pico.

    Activation: ReLU for hidden, Softmax for output
    """

    def __init__(self, input_size=20, hidden_size=10, output_size=6):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size

        # Initialize weights (small random values)
        # In production, these would be loaded from training
        self.W1 = [[0.0] * input_size for _ in range(hidden_size)]
        self.b1 = [0.0] * hidden_size
        self.W2 = [[0.0] * hidden_size for _ in range(output_size)]
        self.b2 = [0.0] * output_size

        # Initialize with small random values
        import random
        for i in range(hidden_size):
            for j in range(input_size):
                self.W1[i][j] = (random.random() - 0.5) * 0.2
        for i in range(output_size):
            for j in range(hidden_size):
                self.W2[i][j] = (random.random() - 0.5) * 0.2

    @staticmethod
    def _relu(x):
        return max(0, x)

    @staticmethod
    def _softmax(x):
        # Numerically stable softmax
        max_x = max(x)
        exp_x = [math.exp(xi - max_x) for xi in x]
        sum_exp = sum(exp_x)
        return [e / sum_exp for e in exp_x]

    def forward(self, features):
        """Forward pass through network."""
        if features is None or len(features) != self.input_size:
            return [1/6] * 6

        # Hidden layer
        hidden = []
        for i in range(self.hidden_size):
            z = self.b1[i]
            for j in range(self.input_size):
                z += self.W1[i][j] * features[j]
            hidden.append(self._relu(z))

        # Output layer
        output = []
        for i in range(self.output_size):
            z = self.b2[i]
            for j in range(self.hidden_size):
                z += self.W2[i][j] * hidden[j]
            output.append(z)

        return self._softmax(output)

    def predict(self, features):
        """Predict class."""
        proba = self.forward(features)
        return proba.index(max(proba))

    def predict_proba(self, features):
        """Return class probabilities."""
        return self.forward(features)

    def save_model(self):
        """Export weights as string."""
        lines = [f"NN,{self.input_size},{self.hidden_size},{self.output_size}"]

        # W1
        for row in self.W1:
            lines.append(','.join(f"{w:.6f}" for w in row))
        # b1
        lines.append(','.join(f"{b:.6f}" for b in self.b1))
        # W2
        for row in self.W2:
            lines.append(','.join(f"{w:.6f}" for w in row))
        # b2
        lines.append(','.join(f"{b:.6f}" for b in self.b2))

        return '\n'.join(lines)

    def load_model(self, data):
        """Load weights from string."""
        lines = data.strip().split('\n')
        header = lines[0].split(',')
        self.input_size = int(header[1])
        self.hidden_size = int(header[2])
        self.output_size = int(header[3])

        idx = 1
        # W1
        self.W1 = []
        for i in range(self.hidden_size):
            self.W1.append([float(x) for x in lines[idx].split(',')])
            idx += 1
        # b1
        self.b1 = [float(x) for x in lines[idx].split(',')]
        idx += 1
        # W2
        self.W2 = []
        for i in range(self.output_size):
            self.W2.append([float(x) for x in lines[idx].split(',')])
            idx += 1
        # b2
        self.b2 = [float(x) for x in lines[idx].split(',')]


# ==============================================================================
# DATA COLLECTOR FOR TRAINING
# ==============================================================================

class TrainingDataCollector:
    """
    Collect labeled training data for ML models.

    Usage:
        collector = TrainingDataCollector()

        # Collect FORWARD samples
        collector.start_recording(FORWARD)
        for _ in range(100):
            collector.add_sample(features)
        collector.stop_recording()

        # Export for PC training
        csv_data = collector.export_csv()
    """

    def __init__(self):
        self.data = []  # List of (features, label)
        self.recording = False
        self.current_label = None

    def start_recording(self, label):
        """Start recording samples with given label."""
        self.recording = True
        self.current_label = label
        print(f"Recording {CLASS_NAMES[label]}...")

    def stop_recording(self):
        """Stop recording."""
        self.recording = False
        print(f"Stopped. Total samples: {len(self.data)}")

    def add_sample(self, features):
        """Add sample if recording."""
        if self.recording and features is not None:
            self.data.append((features[:], self.current_label))

    def get_sample_counts(self):
        """Get count per class."""
        counts = [0] * 6
        for _, label in self.data:
            counts[label] += 1
        return counts

    def export_csv(self):
        """Export data as CSV string."""
        if not self.data:
            return ""

        # Header
        n_features = len(self.data[0][0])
        header = ','.join([f'f{i}' for i in range(n_features)] + ['label'])

        lines = [header]
        for features, label in self.data:
            feat_str = ','.join(f"{f:.6f}" for f in features)
            lines.append(f"{feat_str},{label}")

        return '\n'.join(lines)

    def clear(self):
        """Clear all data."""
        self.data.clear()
        self.recording = False


# ==============================================================================
# MOTION DETECTOR (HIGH-LEVEL API)
# ==============================================================================

class MotionDetector:
    """
    High-level motion detection API combining all methods.

    Usage:
        detector = MotionDetector()
        detector.set_method('knn')  # or 'rules', 'nn'

        # Add IMU sample
        detector.add_sample(ax, ay, az, gx, gy, gz)

        # Get prediction
        movement = detector.predict()
        print(CLASS_NAMES[movement])
    """

    def __init__(self, method='rules', window_size=50):
        self.feature_extractor = FeatureExtractor(window_size)

        # Available classifiers
        self.classifiers = {
            'rules': RuleBasedClassifier(),
            'knn': KNNClassifier(k=5),
            'nn': SimpleNeuralNetwork(),
        }

        self.current_method = method
        self.last_prediction = STOPPED
        self.last_confidence = 0.0

    def set_method(self, method):
        """Set classification method: 'rules', 'knn', or 'nn'."""
        if method in self.classifiers:
            self.current_method = method

    def add_sample(self, ax, ay, az, gx, gy, gz):
        """Add new sensor sample."""
        self.feature_extractor.add_sample(ax, ay, az, gx, gy, gz)

    def predict(self):
        """Get movement prediction."""
        features = self.feature_extractor.extract_features()
        classifier = self.classifiers[self.current_method]

        self.last_prediction = classifier.predict(features)
        proba = classifier.predict_proba(features)
        self.last_confidence = max(proba) if proba else 0

        return self.last_prediction

    def predict_proba(self):
        """Get class probabilities."""
        features = self.feature_extractor.extract_features()
        return self.classifiers[self.current_method].predict_proba(features)

    def get_class_name(self, class_id=None):
        """Get name of class."""
        if class_id is None:
            class_id = self.last_prediction
        return CLASS_NAMES[class_id]

    def train_knn(self, features, label):
        """Add training sample to KNN."""
        self.classifiers['knn'].add_training_sample(features, label)

    def get_features(self):
        """Get current feature vector."""
        return self.feature_extractor.extract_features()
