# KOLAMeRo
### Adaptive und benutzerfreundliche Kollaboration von Menschen und autonomen mobilen Robotern durch kontinuierlich lernende Algorithmen

Die Kollaboration von Mensch und Roboter ist ein essentieller Baustein für deren Akzeptanz und flexiblen Einsatz.
Die Interaktion von Mensch und mobilem Roboterhält bereits grundlegend Einzug in die Intralogistik. 
Das Ziel des Forschungsvorhabens ist es, die Kollaboration von Menschen und autonomen mobilen Robotern benutzerfreundlicher und intuitiver zu gestalten.

- Demonstrator mit multimodalen Schnittstellen zur Interaktion und Kollaboration
   -Gesten-und Sprachsteuerung
   -Blickbewegung
   - Projektoren am Roboter
   - Geeignete Umgebungserfassung
   - Wissensdatenbank und -manage

- Kontinuierliches Lernen
   - zur Flexibilisierung und automatisierten Adaption der Kollaboration
   - Optimierung bzw. Adaption von Algorithmen, z. B. zur Gesteninterpretation
   - Adaption von Verhaltensweisen des Roboters durch den Nutzer
   - Erweiterung und Anpassung der Wissensdatenbank

- Nutzerstudien zur Bewertung der Interaktion und Kollaboration
   - Verifizierung und Bewertung der Kollaboration
   - Untersuchung der Verbesserung der Kollaboration durch das kontinuierliche Lernen


### Ziele des Projekts:
-  Erarbeitung und systematische Untersuchung von
multimodalen Kollaborationstechniken für mobile Roboter
- Optimierung der Nutzerfreundlichkeit und intuitiven
Kollaboration durch die Möglichkeit des kontinuierlichen
Lernens des mobilen Roboters
- Bewertung des Einsatzes von kontinuierlichem Lernen in der
Intralogistik und Erarbeitung von Handlungsempfehlungen

## Nutzen des Forschungsprojektes:
- Ermöglichung der Nutzung von mobilen Robotern auch bei
komplexen Arbeitsprozessen, einschließlich Kollaboration
- Akzeptanzsteigerung für den Einsatz und die Kollaboration
- Einblick in Ansätze zum kontinuierlichen Lernen
- Katalog mit Handlungsempfehlungen zur Weiterentwicklung
der Kollaboration mit mobilen Robotern

### Rahmenbedingungen:
Definierte Szenarien:
−Intuitive Inbetriebnahme
−Soziale Navigation
−Verbesserte Kommunikation   
Betrachtung eines autonomen mobilen Roboters, Keine Entwicklung eines Assistenzsystems

### Wissenschaftliche Fragestellungen:
- Wie sollte die multimodale Kollaboration zwischen Mensch
und autonomen mobilen Robotern gestaltet werden?   
- Wie muss das Wissens eines Roboters strukturiert und
gespeichert werden, um kontinuierliches Lernen zu
ermöglichen?   
- Wie kann mit kontinuierlichem Lernen die Kollaboration
zwischen Mensch und mobilem Roboter intuitiver und
nutzerfreundlicher gestaltet werden?

# gesture recognition

### Dynamic hand-gesture-recognition-using-mediapipe
Estimate hand pose using MediaPipe (Python version).<br> 
This repository contains the following contents.

* Dynamic gesture recognition model(TFLite)
* Learning data for hand sign recognition and notebook for model training

### Training
Static and dynamic gesture recognition can add and change training data and retrain the model.

###### Model training
Open "[keypoint_classification.ipynb](keypoint_classification.ipynb)" in Jupyter Notebook and execute from top to bottom.<br>
To change the number of training data classes, change the value of "NUM_CLASSES = 4" <br>and modify the label of "model/keypoint_classifier/keypoint_classifier_label.csv" as appropriate.<br><br>

###### Model structure
The image of the model prepared in "[keypoint_classification.ipynb](keypoint_classification.ipynb)" is as follows.
<img src="https://user-images.githubusercontent.com/37477845/102246723-69c76a00-3f42-11eb-8a4b-7c6b032b7e71.png" width="50%"><br><br>

### Reference
* [MediaPipe](https://mediapipe.dev/)
* Kazuhito Takahashi(https://twitter.com/KzhtTkhs)

