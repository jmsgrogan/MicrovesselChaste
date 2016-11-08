def get_class(className):
    
    """ return a class instance from a naming string
    """ 
    
    parts = className.split('.')
    module = ".".join(parts[:-1])
    m = __import__( module )
    for comp in parts[1:]:
        m = getattr(m, comp)            
    return m