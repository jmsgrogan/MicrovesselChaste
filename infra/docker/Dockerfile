 FROM continuumio/miniconda
 RUN apt-get install build-essential lsb-release liblapack-dev libblas-dev libatlas-base-dev -y
 RUN conda config --add channels conda-forge
 RUN conda config --add channels jmsgrogan
 RUN conda install microvessel-chaste

 # Add Tini. Tini operates as a process subreaper for jupyter. This prevents
 # kernel crashes.
 ENV TINI_VERSION v0.6.0
 ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /usr/bin/tini
 RUN chmod +x /usr/bin/tini
 ENTRYPOINT ["/usr/bin/tini", "--"]

 EXPOSE 8888
 CMD ["jupyter", "notebook", "--port=8888", "--no-browser", "--ip=0.0.0.0"]
