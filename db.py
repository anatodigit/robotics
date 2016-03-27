#!/usr/bin/python
import MySQLdb

class MyDB(object):
    _db_connection = None
    _db_cur = None
    
    def __init__(self):
        self._db_connection = MySQLdb.connect('localhost','robot','1234','robot')
        self._db_cur = self._db_connection.cursor()
        
    def query(self, query, params):
        return self._db_cur.execute(query, params)
        
    def __del__(self):
        self._db_connection.close()
        