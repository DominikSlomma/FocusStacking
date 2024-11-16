import bioformats
import javabridge
import numpy as np
import imageio

# Startet die Java-Umgebung
javabridge.start_vm(class_path=bioformats.JARS)

def read_vsi(file_path, output_folder):
    # Öffne die VSI-Datei
    with bioformats.ImageReader(file_path) as reader:
        # Bestimme die Anzahl der Serien (Kanal, Zeit, Z-Stack)
        num_series = reader.rdr.getSeriesCount()
        
        for series_idx in range(num_series):
            # Wähle die Serie aus (z. B. Z-Stack oder Zeit)
            reader.setSeries(series_idx)
            
            # Bestimme die Anzahl der Bilder in der Serie (Z-Stack oder Zeitpunkte)
            num_images = reader.rdr.getImageCount()
            
            for i in range(num_images):
                # Lese das Bild als NumPy-Array
                image = reader.read(series=series_idx, index=i, rescale=False)
                
                # Speichere das Bild als PNG oder TIFF
                output_path = f"{output_folder}/series_{series_idx}_image_{i}.png"
                imageio.imwrite(output_path, image)
                print(f"Bild {i} in Serie {series_idx} wurde gespeichert als: {output_path}")
    
    print("Alle Bilder wurden erfolgreich extrahiert.")

# Pfad zur .vsi Datei und der Zielordner zum Speichern
vsi_file = "/home/anonym/Schreibtisch/PhD/code/FocusStacking/data/vsiTest/Image_681.vsi"
output_folder = "/home/anonym/Schreibtisch/PhD/code/FocusStacking/data/vsiTest/output"

# Funktion aufrufen
read_vsi(vsi_file, output_folder)

# Java-Umgebung beenden
javabridge.kill_vm()
