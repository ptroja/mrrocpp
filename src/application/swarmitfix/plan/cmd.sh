# Generowanie parsera C++ na podstawie XML Schema
~/ports/xsd-3.3.0-i686-linux-gnu/bin/xsd cxx-tree \
	--function-naming lcc \
	--type-regex '/ (.+)/\u$1_t/' \
	--root-element plan \
	plan.xsd

# Uproszczenie pliku z formatu exportowanego przez planistow
xsltproc preprocess.xsl PlanXML1.xml
