bledy systemu operacyjnego !!!:
	1) komunikacja sieciowa - spawn dziwnie dziala w sieci.
	2) signal wysylany do watku edp_master odbierany jest rowniez przez watek edp_serwo
	05.08.07 - nie zawsze dziala ustawianie flagi PT_GHOST z funkcji srodowiska app_builder


bledy aplikacji  + priorytet (1 - 10):
	04.01.07 - wyeliminowac redukcje priorytetu procesu odbierajacego wiadomosc poprzez modyfikacje sposobu zakladania kanalu - p 6
	05.02.03 - przerwy w generowaniu przerwan przez karte nr 1 ze sterownika irp6 postument
	05.08.05 - przywieszanie komunikacji miedzyprocesowej na wezle z edp_speaker
	05.08.06 - wyciek pamieci VSP wizyjnego
	05.11.18 - sygnal, ktory jest wysylany do ECP przez MP nie jest obslugiwany w ECP, kiedy ECP zawiesza sie na MsgSend do UI w funkcji operator_reaction
	06.02.09 - dziwaczne zachowanie kontrolerow osi po poleceniu finish_synchro - konieczna inicjacja trybu zerowego 
		do niezawodnego przyjmowania nowych polecen ruchu


wskazane modifykacje  + priorytet (1 - 10):
	04.11.07 - strumieniowa obsluge plikow ini z dynamicznym przydzialem pamieci, listami etc. (ew. XML) - p 1
	05.08.17 - dopracowac obsluge menu (przejscie miedzy kolejnymi menu etc.) - p 3
	05.08.21 - dodac obsluge wyjatkow w nowo-implementowanych funkcjach (w szczegolnosci obsluga sily) - p 4
	07.02.01 - uruchomic krzywe nurbs w euler i angle_axis z chwytakiem a takze w joints i motors - p 8
	07.02.01 - zastapic unie IMAGE w SENSOR.H przez odpowiednia hierarchie polimorficznych klas buforow komunikacyjnych - p 1
	07.02.07 - wstawic enkodery w robocie Irp6_on_track 
					uklady rezolwerowe wprowdzaja blad niesystematyczny w pomiarze polozenia !!!! - p 10
	07.02.16 - wprowadzic w sprzecie uruchamianie ruchow recznych poprzez regulatory scalone
		 w sytuacji gdy nowe sterowanie z PC nie przychodzi w ustalonym czasie (np 10ms), ruchy reczne  - p 10
	07.06.15 - blad  transfomacji sil na poziomie samego czujnika ATI6284 (rozwazyc prace inzynierska) - p 10
