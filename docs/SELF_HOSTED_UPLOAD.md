# Sauvegarde privée des trajets

La branche contient le processus optionnel `home_uploader`. Sans fichier de
configuration il reste inactif et ne change pas le comportement du comma.

Créer `/data/comma_home_upload.json` sur le comma :

```json
{
  "server_url": "https://obd.rudder-aero.com",
  "token": "LE_MEME_JETON_QUE_SUR_LE_SERVEUR",
  "chunk_bytes": 4194304,
  "min_age_seconds": 90,
  "verify_tls": true,
  "upload_video": true,
  "upload_logs": true
}
```

Le fichier doit appartenir à `comma` et rester lisible uniquement par ce
compte (`chmod 600`). Un redémarrage de l'application suffit ensuite.

## Comportement

- l'envoi ne démarre que lorsque `IsOffroad` est vrai et que la connexion est
  de type Wi-Fi ;
- les segments contenant un fichier `.lock` et les fichiers modifiés depuis
  moins de 90 secondes sont ignorés ;
- `rlog` et `qlog` sont compressés en Zstandard avant l'envoi ;
- les vidéos caméra, `rlog` et `qlog` sont envoyés par blocs avec reprise à
  l'octet confirmé par le serveur ;
- les originaux restent sur le comma. Le processus ne les efface jamais ;
- l'état local se trouve dans `/data/comma-home-uploader/state.json`.

Les trajets reçus sont visibles dans **Trajets comma** sur l'application OBD.
Les MP4 générés par le serveur sont lisibles depuis Safari sur iPad. Les
journaux `rlog.zst` conservent les trames CAN complètes du trajet.

Depuis le dépôt OBD, le fichier peut être installé sans afficher le jeton dans
l'historique du shell :

```bash
scripts/configure_comma_home_upload.sh \
  ADRESSE_DU_COMMA \
  https://obd.rudder-aero.com \
  JETON_DU_SERVEUR
```
