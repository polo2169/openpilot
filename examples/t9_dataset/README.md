# Exemple de chronologie T9

`timeline.example.csv` est un exemple **synthétique**, sans donnée issue d'un
véhicule. Il montre les champs utiles pour expliquer une pause latérale sur
clignotant ou effort conducteur, puis la reprise après 0,5 seconde de lignes
fiables.

Les datasets réels ne sont pas versionnés dans Git. Ils sont produits sous
`data/`, qui est ignoré, avec `export_t9_dataset.py`. L'archive contient les
segments `cereal.Event` bruts, leur manifeste SHA-256 et le code T9 associé.

`truck_approach.sample.csv` est un petit extrait **réel et décodé** de
l'approche camion étudiée avant l'installation de la version EPS finale. Il ne
contient ni GPS, VIN, image, identifiant d'appareil, CAN brut ou identifiant de
route. La source brute avait l'empreinte SHA-256
`4e64aa615c736774aadcd814cd073404aa4ccfcf0903fc8ad9b1db77ed3fc023`.
Les colonnes `old_*` et `new_*` comparent deux calculs hors ligne sur les mêmes
détections enregistrées ; `new_*` ne représente pas un second trajet réel.
