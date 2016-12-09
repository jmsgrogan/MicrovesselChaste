 FROM continuumio/miniconda
 RUN apt-get install build-essential lsb-release libgl1-mesa-dev:amd64 libxt-dev:amd64 libatlas-base-dev xvfb -y && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
 RUN conda config --add channels conda-forge && conda config --add channels jmsgrogan
 RUN conda install microvessel-chaste && conda install libgcc --force && conda clean --yes --all && rm -rf /opt/conda/pkgs/*
 #RUN conda install vtk jupyter && conda install libgcc && conda clean --yes --all && rm -rf /opt/conda/pkgs/*

 # Add Tini. Tini operates as a process subreaper for jupyter. This prevents
 # kernel crashes.
 ENV TINI_VERSION v0.6.0
 ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /usr/bin/tini
 RUN chmod +x /usr/bin/tini 

 ENTRYPOINT ["/usr/bin/tini", "--"]

 CMD xvfb-run --server-args="-screen 0 1024x768x24" jupyter notebook --port=8888 --no-browser --ip=0.0.0.0
