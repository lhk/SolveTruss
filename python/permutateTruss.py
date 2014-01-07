import json
import zlib

with open('pool/truss.z', 'rb') as file: 
    Truss = json.loads(zlib.decompress(file.read()))
    



with open('pool/truss.z', 'wb') as file: 
    file.write(zlib.compress(json.dumps(Truss)))