import argparse
import logging
import chaste.projects.angiogenesis.utility.readwrite

class SimpleIOBase(object):
    
    """
    Class for ensuring consistency in the definition of simple tools with a single input and single output.
    
    Classes defining operations with low numbers of inputs can inherit from this one and overwrite the 
    update method. Child classes can be run in standalone mode if suitable readers and writers have been
    implemented for input and output.
    
    @param self.input the input to the tool
    @return self.output the tool output 
    """
    
    def __init__(self):
        self.input = None
        self.output = None
        self.tool_name = "Default"
        
    def set_input(self, my_input):
        self.input = my_input
        
    def update(self):
        pass
    
    def get_output(self):
        return self.output
    
    def run_standalone(self, sys_args):
        parser = argparse.ArgumentParser()
        parser.add_argument("-i","--input", help="Input file path",type=str)
        parser.add_argument("-o", "--output", help="Output file path",type=str)
        args = parser.parse_args()
        logging.info("Starting: " + self.tool_name)
        logging.info("Reading: " + args.input)
        self.input = chaste.projects.angiogenesis.utility.readwrite.read(args.input)
        logging.info("Running: " + self.tool_name)
        self.update()
        logging.info("Writing: " + args.output)
        chaste.projects.angiogenesis.utility.readwrite.write(self.output, args.output)
        logging.info("Finished: " + self.tool_name)
    