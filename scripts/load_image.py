import sys
from cStringIO import StringIO

from bson.binary import Binary
from pymongo import MongoClient
from PIL import Image


client = MongoClient()
db = client.so

for rec in db['db'].find():
    im = Image.open(StringIO(rec['data']))
    im.show()