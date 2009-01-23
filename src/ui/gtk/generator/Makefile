XML_FILES = $(wildcard *.xml)
XSL_FILES = $(wildcard *.xsl)
XSD_FILES = $(wildcard *.xsd)

TIMESTAMP_FILE = ../timestamp.xml

all: $(TIMESTAMP_FILE)

$(TIMESTAMP_FILE): $(XML_FILES) $(XSL_FILES) $(XSD_FILES)
	xsltproc robots.xml --noout --xinclude

clean:
	rm -rf ../timestamp.xml
