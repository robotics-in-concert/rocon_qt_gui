import QtQuick 1.0

Item {
    id:joyStick;
    property int offset:30;
    property string direction:"C";
    property bool isPressedHover:false;
    
    signal mousePressed();
    signal mouseReleased();
    signal feedback(double x, double y);

    Rectangle {
        id:totalArea
        color:"gray"
        radius: parent.width/2
        opacity: 0.5
        width:parent.width
        height:parent.height
    }

    Rectangle{
        id:stick
        width:totalArea.width/2
        height: totalArea.height/2
        radius: height/2
        x: totalArea.width/4;
        y: totalArea.height/4;        
        color:"black"
    }

    MouseArea{
        id:mouseArea
        anchors.fill: parent
        
        onMousePositionChanged: {            
         var rad = totalArea.radius;
         rad =  rad * rad;

         // calculate distance in x direction
         var xDist = mouseX - (totalArea.x + totalArea.radius);
         xDist = xDist * xDist;

         // calculate distance in y direction
         var yDist = mouseY - (totalArea.y + totalArea.radius);
         yDist = yDist * yDist;

         //total distance for inner circle
         var dist = xDist + yDist;            

         //if distance if less then radius then inner circle is inside larger circle
         if( rad < dist) {
             return;
         }

         //center of larger circle
         var oldX = stick.x; var oldY = stick.y;
         stick.x = mouseX - stick.radius;
         stick.y = mouseY - stick.radius;

         // send back x, y transformed onto [-1.0, 1.0] with x, y flipped to be human sensible
         joyStick.feedback((mouseX - totalArea.width/2.0)/(totalArea.width/2.0), (totalArea.height/2.0 - mouseY)/(totalArea.height/2.0));

        }
       
        onPressed: {
           joyStick.isPressedHover = true;
           mousePressed()
        }

        onReleased: {
            //snap to center
            stick.x = totalArea.width /4;
            stick.y = totalArea.height/4;
            joyStick.isPressedHover = false;
            mouseReleased()
        }
    }
}
