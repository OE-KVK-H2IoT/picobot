#!/usr/bin/env python3
"""
==============================================================================
MOTION CLASSIFIER - AI-Based Motion Classification
==============================================================================

This module provides machine learning tools for classifying robot motion
states from sensor data.

MACHINE LEARNING OVERVIEW:
--------------------------
Machine learning allows computers to learn patterns from data rather than
being explicitly programmed. For motion classification:

1. TRAINING: Feed the model labeled examples
   - "This sensor data = FORWARD"
   - "This sensor data = TURN_LEFT"
   - The model learns patterns that distinguish each motion type

2. INFERENCE: Use the trained model on new data
   - Given new sensor readings, predict the motion type
   - Model recognizes patterns it learned during training

CLASSIFICATION METHODS:
-----------------------
This module includes several approaches:

1. Rule-Based: Simple thresholds (no ML, good baseline)
2. K-Nearest Neighbors (KNN): Find similar training samples
3. Decision Tree: Learn decision rules from data
4. Random Forest: Ensemble of decision trees (most robust)
5. Neural Network: Deep learning approach (optional, needs more data)

FEATURES USED:
--------------
The classifier uses these sensor features:
  - ax, ay, az: Accelerometer (motion direction, tilt)
  - gx, gy, gz: Gyroscope (rotation, turning)
  - vib: Vibration level (speed indicator)
  - bumps: Bump count change (movement indicator)

LABELS:
-------
Motion classes to predict:
  - STOPPED: Robot not moving
  - FORWARD: Moving forward
  - BACKWARD: Moving backward
  - TURN_LEFT: Rotating left
  - TURN_RIGHT: Rotating right

USAGE:
------
Training:
    python motion_classifier.py train ai_dataset.csv --output model.pkl

Prediction:
    python motion_classifier.py predict model.pkl recording.csv

Evaluation:
    python motion_classifier.py evaluate ai_dataset.csv

As module:
    from analyzers import MotionClassifier
    clf = MotionClassifier()
    clf.train_from_csv('ai_dataset.csv')
    prediction = clf.predict({'ax': 0.1, 'gx': 0, ...})

==============================================================================
"""

import csv
import json
import pickle
import argparse
import math
from collections import Counter


class MotionClassifier:
    """
    Classifies robot motion from sensor data.

    Supports multiple classification algorithms from simple rule-based
    to machine learning approaches.

    Attributes:
        model_type (str): Type of model ('rule', 'knn', 'tree', 'forest')
        model: Trained model object (sklearn model or custom)
        feature_names (list): Names of features used for classification
        label_names (list): Possible motion labels
    """

    # Feature columns used for classification
    FEATURE_NAMES = ['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'vib', 'bumps']

    # Motion labels
    LABELS = ['STOPPED', 'FORWARD', 'BACKWARD', 'TURN_LEFT', 'TURN_RIGHT']

    def __init__(self, model_type='rule'):
        """
        Initialize the motion classifier.

        Args:
            model_type: Classification algorithm to use
                'rule': Simple threshold-based (no training needed)
                'knn': K-Nearest Neighbors (needs sklearn)
                'tree': Decision Tree (needs sklearn)
                'forest': Random Forest (needs sklearn, recommended)
        """
        self.model_type = model_type
        self.model = None
        self.feature_names = self.FEATURE_NAMES.copy()
        self.label_names = self.LABELS.copy()
        self.training_data = []
        self.is_trained = False

    def _extract_features(self, sample):
        """
        Extract feature vector from sample dictionary.

        Args:
            sample: Dict with sensor values

        Returns:
            list: Feature values in consistent order
        """
        features = []
        for name in self.feature_names:
            val = sample.get(name, 0)
            # Handle string values (from CSV)
            if isinstance(val, str):
                try:
                    val = float(val)
                except ValueError:
                    val = 0.0
            features.append(float(val))
        return features

    def load_training_data(self, filename):
        """
        Load labeled training data from CSV.

        Expected format: label,ax,ay,az,gx,gy,gz,us,motor_l,motor_r,bumps,vib

        Args:
            filename: Path to CSV file

        Returns:
            tuple: (features_list, labels_list)
        """
        features = []
        labels = []

        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                label = row.get('label', 'UNKNOWN')
                if label not in self.LABELS:
                    continue

                feat = self._extract_features(row)
                features.append(feat)
                labels.append(label)

        print(f"Loaded {len(features)} training samples from {filename}")

        # Store for later use
        self.training_data = list(zip(features, labels))

        return features, labels

    def load_training_json(self, filename):
        """
        Load labeled training data from JSON.

        Expected format: {"FORWARD": [{...}, ...], "BACKWARD": [...], ...}

        Args:
            filename: Path to JSON file

        Returns:
            tuple: (features_list, labels_list)
        """
        features = []
        labels = []

        with open(filename, 'r') as f:
            data = json.load(f)

        for label, samples in data.items():
            if label not in self.LABELS:
                continue
            for sample in samples:
                feat = self._extract_features(sample)
                features.append(feat)
                labels.append(label)

        print(f"Loaded {len(features)} training samples from {filename}")
        self.training_data = list(zip(features, labels))

        return features, labels

    # =========================================================================
    # RULE-BASED CLASSIFIER (No training needed)
    # =========================================================================

    def predict_rule_based(self, sample):
        """
        Classify motion using simple rules.

        This is a baseline approach that uses threshold-based rules.
        It works without any training data.

        Rules:
          - Low vibration + low gyro = STOPPED
          - High gz (yaw rate) = turning
          - High ax (forward accel) = forward/backward

        Args:
            sample: Dict with sensor values

        Returns:
            str: Predicted motion label
        """
        ax = float(sample.get('ax', 0))
        gx = float(sample.get('gx', 0))
        gy = float(sample.get('gy', 0))
        gz = float(sample.get('gz', 0))
        vib = float(sample.get('vib', 0))

        # Calculate gyro magnitude (rotation intensity)
        gyro_mag = math.sqrt(gx**2 + gy**2 + gz**2)

        # Thresholds (tune these for your robot)
        STOPPED_VIB = 2.0      # Vibration below this = stopped
        TURN_RATE = 15.0       # Gyro above this = turning
        ACCEL_THRESHOLD = 0.1  # Accel above this = moving

        # Rule 1: Check if stopped
        if vib < STOPPED_VIB and gyro_mag < 5.0:
            return 'STOPPED'

        # Rule 2: Check for turning (high yaw rate)
        if abs(gz) > TURN_RATE:
            return 'TURN_LEFT' if gz > 0 else 'TURN_RIGHT'

        # Rule 3: Check for forward/backward (forward acceleration)
        # Assuming X-axis is forward
        if abs(ax) > ACCEL_THRESHOLD:
            return 'FORWARD' if ax > 0 else 'BACKWARD'

        # Rule 4: If vibrating but no clear direction = forward (default)
        if vib > STOPPED_VIB:
            return 'FORWARD'

        return 'STOPPED'

    # =========================================================================
    # K-NEAREST NEIGHBORS (Simple ML approach)
    # =========================================================================

    def train_knn(self, features, labels, k=5):
        """
        Train a K-Nearest Neighbors classifier.

        KNN is one of the simplest ML algorithms:
        1. Store all training samples
        2. For new sample, find K closest training samples
        3. Predict the most common label among those K samples

        Args:
            features: List of feature vectors
            labels: List of corresponding labels
            k: Number of neighbors to consider

        Note: This is a simple implementation. For production, use sklearn.
        """
        try:
            from sklearn.neighbors import KNeighborsClassifier
            self.model = KNeighborsClassifier(n_neighbors=k)
            self.model.fit(features, labels)
            self.is_trained = True
            print(f"Trained KNN with k={k} on {len(features)} samples")
        except ImportError:
            # Fallback: store data for manual KNN
            self.training_data = list(zip(features, labels))
            self.model = {'k': k, 'data': self.training_data}
            self.is_trained = True
            print(f"Trained simple KNN (no sklearn) with k={k}")

    def predict_knn(self, sample):
        """
        Predict using K-Nearest Neighbors.

        Args:
            sample: Dict with sensor values

        Returns:
            str: Predicted label
        """
        features = self._extract_features(sample)

        # Try sklearn first
        try:
            if hasattr(self.model, 'predict'):
                return self.model.predict([features])[0]
        except:
            pass

        # Manual KNN implementation
        if isinstance(self.model, dict) and 'data' in self.model:
            k = self.model['k']
            data = self.model['data']

            # Calculate distances to all training samples
            distances = []
            for train_feat, train_label in data:
                dist = sum((a - b)**2 for a, b in zip(features, train_feat))
                distances.append((dist, train_label))

            # Sort by distance and get k nearest
            distances.sort(key=lambda x: x[0])
            k_nearest = distances[:k]

            # Vote for most common label
            labels = [label for _, label in k_nearest]
            counter = Counter(labels)
            return counter.most_common(1)[0][0]

        return 'UNKNOWN'

    # =========================================================================
    # DECISION TREE (Interpretable ML)
    # =========================================================================

    def train_tree(self, features, labels, max_depth=5):
        """
        Train a Decision Tree classifier.

        Decision trees learn a series of if-then rules:
          if gz > 15: predict TURN_RIGHT
          elif gz < -15: predict TURN_LEFT
          elif vib < 2: predict STOPPED
          ...

        Advantages:
          - Easy to understand and interpret
          - Can visualize the decision process
          - No feature scaling needed

        Args:
            features: List of feature vectors
            labels: List of corresponding labels
            max_depth: Maximum tree depth (prevents overfitting)
        """
        try:
            from sklearn.tree import DecisionTreeClassifier
            self.model = DecisionTreeClassifier(max_depth=max_depth)
            self.model.fit(features, labels)
            self.is_trained = True
            print(f"Trained Decision Tree (depth={max_depth}) on {len(features)} samples")
        except ImportError:
            print("sklearn not available, falling back to rule-based")
            self.model_type = 'rule'

    # =========================================================================
    # RANDOM FOREST (Robust ML - Recommended)
    # =========================================================================

    def train_forest(self, features, labels, n_trees=100, max_depth=10):
        """
        Train a Random Forest classifier.

        Random Forest = ensemble of many decision trees:
          1. Create multiple trees, each trained on random subset of data
          2. Each tree makes a prediction
          3. Final prediction = majority vote of all trees

        This is usually the most robust approach because:
          - Reduces overfitting compared to single tree
          - Handles noise well
          - Works with small datasets

        Args:
            features: List of feature vectors
            labels: List of corresponding labels
            n_trees: Number of trees in forest
            max_depth: Maximum depth per tree
        """
        try:
            from sklearn.ensemble import RandomForestClassifier
            self.model = RandomForestClassifier(
                n_estimators=n_trees,
                max_depth=max_depth,
                random_state=42
            )
            self.model.fit(features, labels)
            self.is_trained = True
            print(f"Trained Random Forest ({n_trees} trees) on {len(features)} samples")
        except ImportError:
            print("sklearn not available, falling back to KNN")
            self.train_knn(features, labels)

    # =========================================================================
    # UNIFIED INTERFACE
    # =========================================================================

    def train(self, features, labels):
        """
        Train the classifier using the configured model type.

        Args:
            features: List of feature vectors
            labels: List of corresponding labels
        """
        if self.model_type == 'rule':
            # Rule-based doesn't need training
            self.is_trained = True
            print("Rule-based classifier ready (no training needed)")
        elif self.model_type == 'knn':
            self.train_knn(features, labels)
        elif self.model_type == 'tree':
            self.train_tree(features, labels)
        elif self.model_type == 'forest':
            self.train_forest(features, labels)
        else:
            print(f"Unknown model type: {self.model_type}")

    def train_from_csv(self, filename):
        """Load data and train in one step."""
        features, labels = self.load_training_data(filename)
        if features:
            self.train(features, labels)

    def train_from_json(self, filename):
        """Load JSON data and train in one step."""
        features, labels = self.load_training_json(filename)
        if features:
            self.train(features, labels)

    def predict(self, sample):
        """
        Predict motion class for a single sample.

        Args:
            sample: Dict with sensor values

        Returns:
            str: Predicted motion label
        """
        if self.model_type == 'rule':
            return self.predict_rule_based(sample)
        elif self.model_type == 'knn':
            return self.predict_knn(sample)
        elif self.model_type in ('tree', 'forest'):
            features = self._extract_features(sample)
            try:
                return self.model.predict([features])[0]
            except:
                return self.predict_rule_based(sample)
        else:
            return self.predict_rule_based(sample)

    def predict_batch(self, samples):
        """
        Predict motion class for multiple samples.

        Args:
            samples: List of sample dictionaries

        Returns:
            list: List of predicted labels
        """
        return [self.predict(s) for s in samples]

    def save_model(self, filename):
        """Save trained model to file."""
        data = {
            'model_type': self.model_type,
            'feature_names': self.feature_names,
            'label_names': self.label_names,
            'is_trained': self.is_trained,
        }

        if self.model_type in ('tree', 'forest', 'knn') and self.model is not None:
            # Use pickle for sklearn models
            with open(filename, 'wb') as f:
                pickle.dump({'meta': data, 'model': self.model}, f)
        else:
            # Use JSON for rule-based or simple models
            with open(filename + '.json', 'w') as f:
                json.dump(data, f, indent=2)

        print(f"Model saved to {filename}")

    def load_model(self, filename):
        """Load trained model from file."""
        try:
            with open(filename, 'rb') as f:
                saved = pickle.load(f)
            self.model = saved['model']
            meta = saved['meta']
            self.model_type = meta['model_type']
            self.feature_names = meta['feature_names']
            self.label_names = meta['label_names']
            self.is_trained = meta['is_trained']
            print(f"Loaded {self.model_type} model from {filename}")
        except:
            print(f"Could not load model from {filename}")

    # =========================================================================
    # EVALUATION
    # =========================================================================

    def evaluate(self, features, labels):
        """
        Evaluate classifier accuracy on test data.

        Args:
            features: List of feature vectors
            labels: List of true labels

        Returns:
            dict: Evaluation metrics
        """
        predictions = []
        for feat in features:
            sample = dict(zip(self.feature_names, feat))
            pred = self.predict(sample)
            predictions.append(pred)

        # Calculate accuracy
        correct = sum(1 for p, t in zip(predictions, labels) if p == t)
        accuracy = correct / len(labels) if labels else 0

        # Per-class metrics
        class_stats = {}
        for label in self.label_names:
            true_positives = sum(1 for p, t in zip(predictions, labels)
                               if p == label and t == label)
            false_positives = sum(1 for p, t in zip(predictions, labels)
                                if p == label and t != label)
            false_negatives = sum(1 for p, t in zip(predictions, labels)
                                if p != label and t == label)

            precision = true_positives / (true_positives + false_positives) \
                       if (true_positives + false_positives) > 0 else 0
            recall = true_positives / (true_positives + false_negatives) \
                    if (true_positives + false_negatives) > 0 else 0
            f1 = 2 * precision * recall / (precision + recall) \
                if (precision + recall) > 0 else 0

            class_stats[label] = {
                'precision': precision,
                'recall': recall,
                'f1': f1,
                'true_positives': true_positives,
                'false_positives': false_positives,
                'false_negatives': false_negatives,
            }

        # Confusion matrix
        confusion = {}
        for true_label in self.label_names:
            confusion[true_label] = {}
            for pred_label in self.label_names:
                count = sum(1 for p, t in zip(predictions, labels)
                          if p == pred_label and t == true_label)
                confusion[true_label][pred_label] = count

        return {
            'accuracy': accuracy,
            'correct': correct,
            'total': len(labels),
            'class_stats': class_stats,
            'confusion': confusion,
        }

    def print_evaluation(self, features, labels):
        """Print formatted evaluation report."""
        results = self.evaluate(features, labels)

        print("\n" + "=" * 60)
        print("MOTION CLASSIFIER EVALUATION")
        print("=" * 60)

        print(f"\nModel Type: {self.model_type}")
        print(f"Overall Accuracy: {results['accuracy']*100:.1f}% "
              f"({results['correct']}/{results['total']})")

        print("\nPer-Class Performance:")
        print(f"  {'Label':12s} {'Precision':>10s} {'Recall':>10s} {'F1':>10s}")
        print("  " + "-" * 44)
        for label in self.label_names:
            stats = results['class_stats'][label]
            print(f"  {label:12s} {stats['precision']:10.1%} "
                  f"{stats['recall']:10.1%} {stats['f1']:10.1%}")

        print("\nConfusion Matrix:")
        print(f"  {'True\\Pred':12s}", end='')
        for label in self.label_names:
            print(f" {label[:7]:>7s}", end='')
        print()
        print("  " + "-" * (12 + 8 * len(self.label_names)))

        for true_label in self.label_names:
            print(f"  {true_label:12s}", end='')
            for pred_label in self.label_names:
                count = results['confusion'][true_label][pred_label]
                print(f" {count:7d}", end='')
            print()

        print("\n" + "=" * 60)


def main():
    """Command-line interface for motion classification."""
    parser = argparse.ArgumentParser(
        description='AI-based motion classification for PicoBot',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Train a model
  python motion_classifier.py train ai_dataset.csv --model forest

  # Evaluate on test data
  python motion_classifier.py evaluate ai_dataset.csv --model forest

  # Use saved model for prediction
  python motion_classifier.py predict model.pkl recording.csv
        """
    )

    subparsers = parser.add_subparsers(dest='command', help='Command')

    # Train command
    train_parser = subparsers.add_parser('train', help='Train a classifier')
    train_parser.add_argument('data_file', help='Training data (CSV or JSON)')
    train_parser.add_argument('--model', '-m', default='forest',
                             choices=['rule', 'knn', 'tree', 'forest'],
                             help='Model type (default: forest)')
    train_parser.add_argument('--output', '-o', default='motion_model.pkl',
                             help='Output model file')

    # Evaluate command
    eval_parser = subparsers.add_parser('evaluate', help='Evaluate classifier')
    eval_parser.add_argument('data_file', help='Test data (CSV or JSON)')
    eval_parser.add_argument('--model', '-m', default='forest',
                            choices=['rule', 'knn', 'tree', 'forest'],
                            help='Model type (default: forest)')

    # Predict command
    pred_parser = subparsers.add_parser('predict', help='Predict from file')
    pred_parser.add_argument('model_file', help='Trained model file')
    pred_parser.add_argument('data_file', help='Data to classify')

    args = parser.parse_args()

    if args.command == 'train':
        clf = MotionClassifier(model_type=args.model)
        if args.data_file.endswith('.json'):
            clf.train_from_json(args.data_file)
        else:
            clf.train_from_csv(args.data_file)
        clf.save_model(args.output)

    elif args.command == 'evaluate':
        clf = MotionClassifier(model_type=args.model)
        if args.data_file.endswith('.json'):
            features, labels = clf.load_training_json(args.data_file)
        else:
            features, labels = clf.load_training_data(args.data_file)
        clf.train(features, labels)
        clf.print_evaluation(features, labels)

    elif args.command == 'predict':
        clf = MotionClassifier()
        clf.load_model(args.model_file)

        # Load data and predict
        samples = []
        with open(args.data_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                samples.append(row)

        predictions = clf.predict_batch(samples)

        # Count predictions
        counts = Counter(predictions)
        print("\nPrediction Results:")
        for label, count in counts.most_common():
            pct = count / len(predictions) * 100
            print(f"  {label}: {count} ({pct:.1f}%)")

    else:
        parser.print_help()


if __name__ == '__main__':
    main()
