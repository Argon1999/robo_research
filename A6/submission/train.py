import os
import cv2
import numpy as np
import csv
import argparse
import pickle
from collections import Counter
from sklearn.model_selection import train_test_split
from sklearn.neighbors import KNeighborsClassifier
from sklearn.svm import SVC
from sklearn.metrics import classification_report, accuracy_score

# Import our shared processing functions
from supplemental.utils import crop_sign, extract_features

def load_data_paths(data_dir):
    """
    Loads all image paths and labels from the data directory.
    Assumes data_dir contains 'labels.txt' and image files.
    """
    all_data = []
    labels_file = os.path.join(data_dir, 'labels.txt')
    if not os.path.isfile(labels_file):
        print(f"Error: 'labels.txt' not found in {data_dir}")
        return []
        
    with open(labels_file, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row: continue
            try:
                img_name = f"{row[0].strip()}.png"
                label = int(row[1].strip())
                img_path = os.path.join(data_dir, img_name)
                
                if os.path.isfile(img_path):
                    all_data.append((img_path, label))
                else:
                    print(f"Warning: Image file not found: {img_path}")
            except Exception as e:
                print(f"Warning: Skipping malformed row: {row}. Error: {e}")
                
    print(f"Found {len(all_data)} images in {data_dir}.")
    return all_data

def preprocess_data(data_list):
    """
    Loads images, crops, extracts features, and returns feature/label arrays.
    
    --- V4 Change ---
    This now SKIPS images where crop_sign returns None (i.e., Class 0).
    The model will only be trained on data from classes 1-5.
    """
    X_features = []
    y_labels = []
    
    print("Processing images and extracting features...")
    for (img_path, label) in data_list:
        image = cv2.imread(img_path)
        if image is None:
            print(f"Warning: Failed to read {img_path}. Skipping.")
            continue
            
        # 1. Crop the image (using shared function)
        cropped_image = crop_sign(image)
        
        # --- V4 Change ---
        if cropped_image is None:
            # This is (most likely) a Class 0 image.
            # We will NOT include it in the SVM training data.
            continue
            
        # 2. Extract features (using shared function)
        features = extract_features(cropped_image)
        
        X_features.append(features)
        y_labels.append(label)
        
    print(f"Successfully processed {len(y_labels)} images.")
    return np.array(X_features), np.array(y_labels)

def main():
    parser = argparse.ArgumentParser(description="Train a sign classifier (KNN or SVM)")
    parser.add_argument('--data_dir', type=str, default='split/train',
                        help="Directory containing training data (from prepare_data.py)")
    parser.add_argument('--val_split', type=float, default=0.2,
                        help="Fraction of training data to use for validation (e.g., 0.2 for 20%)")
    parser.add_argument('--save_path', type=str, default='model.pkl',
                        help="Path to save the trained model file")

    # KNN arguments
    parser.add_argument('--k', type=int, default=7,
                        help="Number of neighbors (K) for KNN")

    args = parser.parse_args()

    # 1. Load data paths and labels
    all_data = load_data_paths(args.data_dir)
    if not all_data:
        return

    # 2. Preprocess all data (crop + extract features)
    X, y = preprocess_data(all_data)
    if args.val_split > 0.0:
        print(f"Splitting data into training and validation ({1.0 - args.val_split} / {args.val_split})...")
        X_train, X_val, y_train, y_val = train_test_split(
            X, y,
            test_size=args.val_split,
            stratify=y,
            random_state=42
        )
        print(f"Train samples: {len(y_train)}, Validation samples: {len(y_val)}")
    else:
        print("Validation split is 0.0. Training on 100% of the data. Skipping validation.")
        X_train, y_train = X, y
        X_val, y_val = None, None # Set validation data to None
        print(f"Train samples: {len(y_train)}")

    # 4. Initialize the model
    print(f"Initializing KNN model with k={args.k}...")
    model = KNeighborsClassifier(n_neighbors=args.k)

    # 5. Train the model
    print(f"Training KNN model...")
    model.fit(X_train, y_train)
    print("Training complete.")

    # 6. Evaluate on the validation set
    # MODIFIED: Only run validation if X_val exists
    if X_val is not None and y_val is not None:
        print("\n--- Validation Results ---")
        y_pred = model.predict(X_val)
        accuracy = accuracy_score(y_val, y_pred)

    
    # 7. Save the model
    print(f"\nSaving trained model to {args.save_path}...")
    with open(args.save_path, 'wb') as f:
        pickle.dump(model, f)
    print("Model saved successfully.")

if __name__ == "__main__":
    main()