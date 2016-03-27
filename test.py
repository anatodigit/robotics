import motors
import db

motors.init()
motors.forward(1)


data = db.fetch_row("SELECT VERSION()")
print "here:"+data[0]