import yaml
import numpy as np
import angles

def load_landmarks(file_path):
    """
    Load a YAML file of landmarks into a Python dictionary.
    
    Args:
        file_path (str): Path to the .yaml file.
    
    Returns:
        dict: Dictionary containing landmarks with their coordinates.
    """
    file=open(file_path, "r")
    data=yaml.safe_load(file)
    lm=data['landmarks']
    
    indexes=lm['id']
    x1=lm['x']
    y1=lm['y']
    landmarks={}
    for i,x,y in zip(indexes,x1,y1):
        landmarks[i]=np.array([x,y])
    return landmarks
    

def custom_residuals(z, z_hat, **kwargs):
    """
    Calculates the measurement residual (z - z_hat) and normalizes the bearing component.
    
    """
    
    # Calculate the raw difference vector
    diff = z - z_hat
    
    # The bearing component is always at index 1 for [range, bearing] measurements.
    BEARING_IDX = 1
    
    #Check if the vector is large enough to contain the bearing index
    if len(diff) > BEARING_IDX:
        
        #Normalize the bearing residual
        diff[BEARING_IDX] = angles.normalize_angle(diff[BEARING_IDX])
        
    return diff

    


