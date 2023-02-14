from pyrosim.commonFunctions import Save_Whitespace

class MATERIAL: 

    def __init__(self, g_value, b_value):
        
        self.depth  = 3

        if g_value == 1.0 and b_value == 0.0:
            self.string1 = '<material name="Green">'

        elif g_value == 0.0 and b_value == 1.0:
            self.string1 = '<material name="Blue">'

        elif g_value == 1.0 and b_value == 1.0:
            self.string1 = '<material name="Cyan">'

        self.string2 = '    <color rgba="0 ' + str(g_value)+ ' ' + str(b_value) + ' 1.0"/>'

        self.string3 = '</material>'

    def Save(self,f):

        Save_Whitespace(self.depth,f)

        f.write( self.string1 + '\n' )

        Save_Whitespace(self.depth,f)

        f.write( self.string2 + '\n' )

        Save_Whitespace(self.depth,f)

        f.write( self.string3 + '\n' )
