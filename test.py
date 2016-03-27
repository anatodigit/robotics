import motors
import db

motors.init()
motors.forward(1)

conn = db.MyDB()

data = conn.fetch_row("SELECT VERSION()")
print "here:"+data[0]