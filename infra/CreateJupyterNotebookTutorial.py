#!/usr/bin/env python


"""Copyright (c) 2005-2017, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


"""Script to convert a tutorial test file into a Jupyter notebook

Notes: 
1. The script starts converting everything after the first #define line
2. The script stops converting after the final #endif.  It is assumed that there
   will be a #if line immediately after the first #define, and a count is kept of
   #if and #endif lines seen in order to allow matching #if/endif blocks within
   the converted section.
3. All C-style block comments '/*' to '*/' are converted to markdown.
4. All other lines, including C++ style comments '//', are kept as code lines
5. In C-block comments (ie markdown), whitespace is removed. Bulleted lists 
   will work but nested bulleted lists won't.
6. To print an empty line in the markdown page, just leave a blank line between paragraphs
   in the comment, as for markdown pages.  The older style of writing EMPTYLINE in a
   paragraph by itself also works, but is deprecated.
7. Lines inside a block comment which start with a '*', i.e.
     /* my comment is
      * two lines long */
   are ok, the initial '*' is removed.
"""


import optparse
import os
import sys
import nbformat as nbf

# This had better match GenerateHowTo.py!
HOWTO_TAG = "HOW_TO_TAG"

def ConvertFileToJupyterNotebook(fileobj, filepath, nb):
    """Convert a single tutorial source file to markdown, returning the page source."""
    # "State machine" state variables
    parsing = False
    ST_NONE, ST_TEXT, ST_CODE, ST_HOWTO = 0, 1, 2, 3 # Status codes
    status = ST_NONE
    in_list = False
    ifdefs_seen = 0
    # Configuration based on file type
    if filepath.endswith('.py'):
        StartsComment = lambda stripped_line: stripped_line.startswith('##')
        IsStillComment = lambda stripped_line: stripped_line.startswith('#')
        EndsComment = lambda stripped_line: not stripped_line.startswith('#')
        CleanEndComment = lambda stripped_line: stripped_line
        end_can_be_start = False # Whether EndsComment and StartsComment can apply to the same line
    else:
        StartsComment = lambda stripped_line: stripped_line.startswith('/*') and not stripped_line.startswith('/**') # Ignore Doxygen comments
        IsStillComment = lambda stripped_line: stripped_line.startswith('*')
        EndsComment = lambda stripped_line: stripped_line.endswith('*/') or stripped_line == '/'
        CleanEndComment = lambda stripped_line: stripped_line[:-2].strip()
        end_can_be_start = True
    
    # SetUp string
    test_setup_string = '# Set up the test \n'
    test_setup_string += 'chaste.cell_based.SetupNotebookTest()\n'
    
    test_teardown_string = '# Tear down the test \n'
    test_teardown_string += 'chaste.cell_based.TearDownNotebookTest()\n'
    
    jupyter_show_first_string = "nb_manager = microvessel_chaste.visualization.JupyterNotebookManager()\n"
    jupyter_show_string = "nb_manager.vtk_show(scene, height=600, width = 1000)\n"
    
    jupyter_parameter_dump_string = "nb_manager.add_parameter_table(file_handler)"
    
    # Output
    last_line = ''
    last_block = ''
    blocks = [] # [string, is_code]
    
    for line in fileobj:
        line = line.rstrip() # Note: also removes '\n'
        # Remove all whitespace and save to a new string.
        # We don't remove the initial whitespace as it will be needed for code lines.
        stripped_line = line.strip()
        
        # We stop processing input after an #endif matching the initial include guard
        if stripped_line.startswith('#endif'):
            assert ifdefs_seen > 0, "#endif seen before #if"
            ifdefs_seen -= 1
            if ifdefs_seen == 0:
                if status is ST_CODE:
                    # close code block
                    blocks.append([last_block, True])
                    last_block = ''
                parsing = False
    
        # If in Parsing mode
        if parsing:
            if status in [ST_TEXT, ST_HOWTO]:
                # We are still in a comment line, so strip it
                line = stripped_line
            
            # Check if the line is a new text line
            comment_started = False
            if StartsComment(stripped_line):
                comment_started = True
                # remove all whitespace and the '/*'
                stripped_line = line = stripped_line[2:].strip()
                # if the last line was code, close the output code block
                if status is ST_CODE:
                    blocks.append([last_block, True])
                    last_block = ''
                # set the status as text
                status = ST_TEXT
            elif status in [ST_TEXT, ST_HOWTO] and IsStillComment(stripped_line):
                # we are in a comment, so get rid of whitespace and the initial '*'
                stripped_line = line = line[1:].strip()
            elif status is ST_NONE and len(stripped_line) > 0:
                # Line has content and isn't a comment => it's code
                blocks.append([last_block, False])
                last_block = ''
                status = ST_CODE
            
            # Check if comment ends
            if EndsComment(stripped_line) and (not comment_started or end_can_be_start):
                # If it's not a Doxygen comment, switch state to unknown
                if status in [ST_TEXT, ST_HOWTO]:
                    # get rid of whitespace and '*/'
                    stripped_line = line = CleanEndComment(stripped_line)
                    status = ST_NONE
            
            # Check for (and strip) HOWTO tagging
            if status is ST_TEXT and stripped_line.startswith(HOWTO_TAG):
                status = ST_HOWTO
            if status is ST_HOWTO:
                if not stripped_line:
                    status = ST_TEXT # Blank comment line ends tagging
                else:
                    stripped_line = line = '' # Strip tag content
            
            if status is ST_TEXT and stripped_line and stripped_line[0] == '*':
                # It's a list, so needs some indentation!
                in_list = True
            if status is ST_TEXT and in_list:
                line = ' '+line
            if in_list and (len(stripped_line) == 0 or status is not ST_TEXT):
                in_list = False
    
            # If the line is a comment just saying 'EMPTYLINE', we'll print a blank line
            if stripped_line == 'EMPTYLINE':
                stripped_line = line = ''
            # We print the line unless we'd get 2 empty lines
            if len(stripped_line) > 0 or len(last_line) > 0:
                last_block += line+'\n'

        # We start processing lines AFTER the first #define..
        if stripped_line.startswith('#define'):
            parsing = True
        if stripped_line.startswith('#if'):
            ifdefs_seen += 1
        last_line = stripped_line
        
    # Assemble the notebook cells
    for eachBlock in blocks:
        if eachBlock[1]:
            
            # Convert the string block to a list for easier processing
            block_list = eachBlock[0].split('\n')
            
            # Strip out class and function calls, unittest and main and left-align all lines
            output_lines = []
            ignore_lines_contain = ["unittest", "__main__", "self.assert"]
            for eachLine in block_list:
                if not any(ignore_string in eachLine for ignore_string in ignore_lines_contain):
                    right_stripped = eachLine.rstrip()
                    if right_stripped[:5] != "class":
                        
                        if "def" in right_stripped:
                            if not len(right_stripped) - len(right_stripped.lstrip(' ')) == 4:
                                if right_stripped[:8].isspace():
                                    output_lines.append(right_stripped[8:])
                                else:
                                    output_lines.append(right_stripped)
                        else:
                            if right_stripped[:8].isspace():
                                output_lines.append(right_stripped[8:])
                            else:
                                output_lines.append(right_stripped)
                    
            # Look for the setup and teardown marks
            for idx, eachLine in enumerate(output_lines):
                output_lines[idx] = output_lines[idx].replace("VtkSceneMicrovesselModifier2()", "JupyterMicrovesselSceneModifier2(nb_manager)")
                output_lines[idx] = output_lines[idx].replace("VtkSceneMicrovesselModifier3()", "JupyterMicrovesselSceneModifier3(nb_manager)")
                if "JUPYTER_SETUP" in eachLine:
                    output_lines[idx] = test_setup_string
                if "JUPYTER_TEARDOWN" in eachLine:
                    output_lines[idx] = test_teardown_string
                if "JUPYTER_SHOW_FIRST" in eachLine:
                    output_lines[idx] = jupyter_show_first_string
                elif "JUPYTER_SHOW" in eachLine:
                    output_lines[idx] = jupyter_show_string
                if "JUPYTER_PARAMETER_DUMP" in eachLine:
                    output_lines[idx] = jupyter_parameter_dump_string
            
            # Reassemble the block as a single string, strip empty lines
            if len(output_lines)>0:
                out_string = "\n".join(output_lines) 
                out_string = os.linesep.join([s for s in out_string.splitlines() if s])
                if out_string.strip():
                    nb['cells'].append(nbf.v4.new_code_cell(out_string))  
        else:
            nb['cells'].append(nbf.v4.new_markdown_cell(eachBlock[0]))                
        
def ConvertTutorialToJupyterNotebook(test_file_path, test_file, other_files, revision=''):
    """Convert a tutorial, possibly comprised of multiple files, to a Jupyter notebook.
    
    test_file is the content of the tutorial test .py file, as an object which will
    return each line in turn when iterated.
    other_files is a list of subsidiary files, which may be empty.  Each entry should
    be a pair, the first item of which is the basename of the file, and the second is
    the contents as for test_file.
    """
    if revision:
        revision = ' at revision r' + str(revision)
        
    nb = nbf.v4.new_notebook()
    nb['cells'] = []

    # Header
    nb['cells'].append(nbf.v4.new_markdown_cell('This tutorial is automatically generated from the file ' + 
                                                test_file_path + revision + '.\n\n'))
    
    # Notebook specific imports
    notebook_header = '# Jupyter notebook specific imports \nimport matplotlib as mpl \nfrom IPython import display \n'
    notebook_header += '%matplotlib inline'
    nb['cells'].append(nbf.v4.new_code_cell(notebook_header))
        
    # Convert each file in turn
    ConvertFileToJupyterNotebook(test_file, test_file_path, nb)

    return nb

def ParseOptions():
    usage = "usage: %prog [options] <test_file>|- <output_file>|-"
    parser = optparse.OptionParser(usage=usage)
    parser.add_option('-r', '--revision', default='',
                      help="Revision")
    parser.add_option('-f', '--real-file-path',
                      help="The real path of the test file, if it is being piped in")
    (options, args) = parser.parse_args()
    
    if len(args) != 2:
        parser.error("You must specify input and output files")
    
    return options, args

if __name__ == '__main__':
    options, args = ParseOptions()
    
    test_file = args[0]
    out_file_name = args[1]
    
    if options.real_file_path:
        real_file_path = options.real_file_path
    else:
        real_file_path = test_file
    
    if test_file == '-':
        # Read from stdin (pipe mode)
        in_file = sys.stdin
    elif test_file[-3:] not in ['.py',] or os.path.basename(test_file)[:4] != 'Test':
        print >>sys.stderr, "Syntax error:", test_file, "does not appear to be a Python test file"
        sys.exit(1)
    else:
        in_file = open(test_file)
    
    if out_file_name == '-':
        # Write to stdout (pipe mode)
        out_file = sys.stdout
    else:
        out_file = open(out_file_name, 'w')
    
    # Do the conversion
    nb = ConvertTutorialToJupyterNotebook(real_file_path, in_file, [], options.revision)
    
    ## Some logging
    print "Generating:" + out_file_name
    
    with open(out_file_name, 'w') as f:
        nbf.write(nb, f)

    # Close files
    if in_file is not sys.stdin:
        in_file.close()

