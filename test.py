import motors
import db

motors.init()
motors.forward(1)
motors.turn_left(1)
motors.reverse(1)
motors.turn_right(1)
motors.forward(1)

conn = db.MyDB()

data = conn.fetch_row("SELECT VERSION()")
print "here:"+data[0]