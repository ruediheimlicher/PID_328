FasdUAS 1.101.10   ��   ��    k             l   g ����  O    g  	  k   f 
 
     l   ��  ��     activate     �    a c t i v a t e      r        m       �      o      ���� $0 xcodeprojektname XcodeProjektname      r        m    	   �      o      ���� 0 filecontents fileContents      r        4    ��  
�� 
alis   l    !���� ! l    "���� " I   �� # $
�� .earsffdralis        afdr #  f     $ �� %��
�� 
rtyp % m    ��
�� 
ctxt��  ��  ��  ��  ��    o      ���� 0 
homeordner     & ' & l   �� ( )��   ( 0 *display dialog "homeordner: " & homeordner    ) � * * T d i s p l a y   d i a l o g   " h o m e o r d n e r :   "   &   h o m e o r d n e r '  + , + l   ��������  ��  ��   ,  - . - r     / 0 / n     1 2 1 m    ��
�� 
ctnr 2 o    ���� 0 
homeordner   0 o      ���� 0 homeordnerpfad   .  3 4 3 l   �� 5 6��   5 2 ,set main to file "datum.c" of homeordnerpfad    6 � 7 7 X s e t   m a i n   t o   f i l e   " d a t u m . c "   o f   h o m e o r d n e r p f a d 4  8 9 8 r    & : ; : n    $ < = < 1   " $��
�� 
pnam = n    " > ? > 2     "��
�� 
file ? o     ���� 0 homeordnerpfad   ; o      ���� 0 dateienliste Dateienliste 9  @ A @ X   ' O B�� C B Z   9 J D E���� D E   9 > F G F o   9 :���� 0 tempname   G m   : = H H � I I  . x c o d e p r o j E k   A F J J  K L K r   A D M N M o   A B���� 0 tempname   N o      ���� $0 xcodeprojektname XcodeProjektname L  O�� O l  E E�� P Q��   P  display dialog tempname    Q � R R . d i s p l a y   d i a l o g   t e m p n a m e��  ��  ��  �� 0 tempname   C o   * +���� 0 dateienliste Dateienliste A  S T S l  P P��������  ��  ��   T  U V U r   P ] W X W b   P Y Y Z Y l  P U [���� [ c   P U \ ] \ o   P Q���� 0 homeordnerpfad   ] m   Q T��
�� 
TEXT��  ��   Z m   U X ^ ^ � _ _  d a t u m . c X o      ���� 0 filepfad   V  ` a ` l  ^ ^�� b c��   b , &display dialog "filepfad: " & filepfad    c � d d L d i s p l a y   d i a l o g   " f i l e p f a d :   "   &   f i l e p f a d a  e f e l  ^ ^�� g h��   g ! tell application "TextEdit"    h � i i 6 t e l l   a p p l i c a t i o n   " T e x t E d i t " f  j k j I  ^ c������
�� .miscactvnull��� ��� obj ��  ��   k  l m l r   d v n o n l  d r p���� p I  d r�� q r
�� .rdwropenshor       file q 4   d j�� s
�� 
file s o   f i���� 0 filepfad   r �� t��
�� 
perm t m   m n��
�� boovtrue��  ��  ��   o o      ���� 0 refnum RefNum m  u v u Q   w� w x y w k   z� z z  { | { r   z � } ~ } l  z � ����  I  z ��� ���
�� .rdwrread****        **** � o   z }���� 0 refnum RefNum��  ��  ��   ~ o      ���� 0 filecontents fileContents |  � � � l  � ���������  ��  ��   �  � � � l  � ��� � ���   � 7 1display dialog "inhalt: " & return & fileContents    � � � � b d i s p l a y   d i a l o g   " i n h a l t :   "   &   r e t u r n   &   f i l e C o n t e n t s �  � � � r   � � � � � n   � � � � � 4   � ��� �
�� 
cpar � m   � �����  � o   � ����� 0 filecontents fileContents � o      ���� 0 datum Datum �  � � � l  � ��� � ���   � &  display dialog "Datum: " & Datum    � � � � @ d i s p l a y   d i a l o g   " D a t u m :   "   &   D a t u m �  � � � r   � � � � � I  � �������
�� .misccurdldt    ��� null��  ��   � o      ���� 	0 heute   �  � � � l  � ��� � ���   � &  display dialog "heute: " & heute    � � � � @ d i s p l a y   d i a l o g   " h e u t e :   "   &   h e u t e �  � � � r   � � � � � n   � � � � � 1   � ���
�� 
year � o   � ����� 	0 heute   � o      ���� 0 jahrtext   �  � � � r   � � � � � n   � � � � � m   � ���
�� 
mnth � o   � ����� 	0 heute   � o      ���� 0 	monattext   �  � � � l  � ��� � ���   � * $display dialog "monat: " & monattext    � � � � H d i s p l a y   d i a l o g   " m o n a t :   "   &   m o n a t t e x t �  � � � r   � � � � � n   � � � � � 7  � ��� � �
�� 
ctxt � m   � ������� � m   � ������� � l  � � ����� � b   � � � � � m   � � � � � � �  0 � n   � � � � � 1   � ���
�� 
day  � o   � ����� 	0 heute  ��  ��   � o      ���� 0 tag   �  � � � l  � ��� � ���   � " display dialog "tag: " & tag    � � � � 8 d i s p l a y   d i a l o g   " t a g :   "   &   t a g �  � � � r   � � � � � J   � � � �  � � � m   � ���
�� 
jan  �  � � � m   � ���
�� 
feb  �  � � � m   � ���
�� 
mar  �  � � � l 	 � � ����� � m   � ���
�� 
apr ��  ��   �  � � � m   � ���
�� 
may  �  � � � m   � ���
�� 
jun  �  � � � m   � ���
�� 
jul  �  � � � m   � ���
�� 
aug  �  � � � l 	 � � ����� � m   � ���
�� 
sep ��  ��   �  � � � m   � ���
�� 
oct  �  � � � m   � ���
�� 
nov  �  ��� � m   � ���
�� 
dec ��   � o      ���� 0 monatsliste MonatsListe �  � � � Y   �5 ��� � ��� � Z  0 � ����� � =   � � � o  	���� 0 	monattext   � n  	 � � � 4  �� �
�� 
cobj � o  ���� 0 i   � o  	���� 0 monatsliste MonatsListe � k  , � �  � � � r  * � � � n  & � � � 7 &�� � �
�� 
ctxt � m  "������ � m  #%������ � l  ����� � b   � � � m   � � � � �  0 � o  ���� 0 i  ��  ��   � o      ���� 	0 monat   �  �� � l +, � � � �  S  +, � - ' wenn true, wird die Schleife verlassen    � � � � N   w e n n   t r u e ,   w i r d   d i e   S c h l e i f e   v e r l a s s e n�  ��  ��  �� 0 i   � m   � ��~�~  � m   ��}�} ��   �  �  � l 66�|�|   &  display dialog "monat: " & monat    � @ d i s p l a y   d i a l o g   " m o n a t :   "   &   m o n a t   r  6M l 	6I�{�z l 6I	�y�x	 n 6I

 7 ;I�w
�w 
cha  m  AC�v�v  m  DH�u�u  l 6;�t�s c  6; o  69�r�r 0 jahrtext   m  9:�q
�q 
ctxt�t  �s  �y  �x  �{  �z   o      �p�p 0 jahr    l NN�o�o   ? 9display dialog "jahr: " & jahr & " jahrtext: " & jahrtext    � r d i s p l a y   d i a l o g   " j a h r :   "   &   j a h r   &   "   j a h r t e x t :   "   &   j a h r t e x t  r  N] n  NY m  UY�n
�n 
nmbr n  NU 2 QU�m
�m 
cha  o  NQ�l�l 0 datum Datum o      �k�k 0 l    l ^^�j !�j    1 +set neuesDatum to text -l thru -13 of Datum   ! �"" V s e t   n e u e s D a t u m   t o   t e x t   - l   t h r u   - 1 3   o f   D a t u m #$# l ^q%&'% r  ^q()( n  ^m*+* 7 am�i,-
�i 
ctxt, m  eg�h�h - m  hl�g�g + o  ^a�f�f 0 datum Datum) o      �e�e 0 
neuesdatum 
neuesDatum& $  Anfang bis und mit Leerschlag   ' �.. <   A n f a n g   b i s   u n d   m i t   L e e r s c h l a g$ /0/ l rr�d12�d  1 2 ,display dialog "neuesDatum A: " & neuesDatum   2 �33 X d i s p l a y   d i a l o g   " n e u e s D a t u m   A :   "   &   n e u e s D a t u m0 454 r  r�676 b  r�898 b  r�:;: b  r�<=< b  r�>?> b  r�@A@ b  r}BCB b  ryDED o  ru�c�c 0 
neuesdatum 
neuesDatumE m  uxFF �GG  "C o  y|�b�b 0 tag  A m  }�HH �II  .? o  ���a�a 	0 monat  = m  ��JJ �KK  .; o  ���`�` 0 jahrtext  9 m  ��LL �MM  "7 o      �_�_ 0 
neuesdatum 
neuesDatum5 NON l ���^PQ�^  P 0 *display dialog "neuesDatum: " & neuesDatum   Q �RR T d i s p l a y   d i a l o g   " n e u e s D a t u m :   "   &   n e u e s D a t u mO STS r  ��UVU b  ��WXW b  ��YZY n  ��[\[ 4  ���]]
�] 
cpar] m  ���\�\ \ o  ���[�[ 0 filecontents fileContentsZ o  ���Z
�Z 
ret X o  ���Y�Y 0 
neuesdatum 
neuesDatumV o      �X�X 0 	neuertext 	neuerTextT ^_^ l ���W`a�W  ` 3 -set paragraph 2 of fileContents to neuesDatum   a �bb Z s e t   p a r a g r a p h   2   o f   f i l e C o n t e n t s   t o   n e u e s D a t u m_ cdc l ���Vef�V  e 9 3display dialog "neues Datum: " & return & neuerText   f �gg f d i s p l a y   d i a l o g   " n e u e s   D a t u m :   "   &   r e t u r n   &   n e u e r T e x td hih I ���Ujk
�U .rdwrseofnull���     ****j o  ���T�T 0 refnum RefNumk �Sl�R
�S 
set2l m  ���Q�Q  �R  i mnm I ���Pop
�P .rdwrwritnull���     ****o o  ���O�O 0 	neuertext 	neuerTextp �Nq�M
�N 
refnq o  ���L�L 0 refnum RefNum�M  n r�Kr I ���Js�I
�J .rdwrclosnull���     ****s o  ���H�H 0 refnum RefNum�I  �K   x R      �G�F�E
�G .ascrerr ****      � ****�F  �E   y I ���Dt�C
�D .rdwrclosnull���     ****t o  ���B�B 0 refnum RefNum�C   v uvu l ���A�@�?�A  �@  �?  v wxw l ���>yz�>  y   Neue Version einsetzen   z �{{ .   N e u e   V e r s i o n   e i n s e t z e nx |}| r  ��~~ m  ���� ���   o      �=�= 0 filecontents fileContents} ��� r  ����� 4  ���<�
�< 
alis� l ����;�:� l ����9�8� I ���7��
�7 .earsffdralis        afdr�  f  ��� �6��5
�6 
rtyp� m  ���4
�4 
ctxt�5  �9  �8  �;  �:  � o      �3�3 0 
homeordner  � ��� l ���2���2  � 0 *display dialog "homeordner: " & homeordner   � ��� T d i s p l a y   d i a l o g   " h o m e o r d n e r :   "   &   h o m e o r d n e r� ��� r  ����� n  ����� m  ���1
�1 
ctnr� o  ���0�0 0 
homeordner  � o      �/�/ 0 homeordnerpfad  � ��� r  ����� n  ����� 1  ���.
�. 
pnam� o  ���-�- 0 homeordnerpfad  � o      �,�, 0 projektname Projektname� ��� l ���+���+  � 2 ,display dialog "Projektname: " & Projektname   � ��� X d i s p l a y   d i a l o g   " P r o j e k t n a m e :   "   &   P r o j e k t n a m e� ��� r  ���� n ���� 1  ��*
�* 
txdl� 1  ���)
�) 
ascr� o      �(�( 0 olddels oldDels� ��� r  ��� m  
�� ���  _� n     ��� 1  �'
�' 
txdl� 1  
�&
�& 
ascr� ��� l �%�$�#�%  �$  �#  � ��� r  ��� n  ��� 2 �"
�" 
citm� o  �!�! 0 projektname Projektname� o      � �  0 zeilenliste Zeilenliste� ��� r  *��� n  &��� m  "&�
� 
nmbr� o  "�� 0 zeilenliste Zeilenliste� o      �� 0 	anzzeilen 	anzZeilen� ��� l ++����  � n hdisplay dialog "Zeilenliste: " & return & (Zeilenliste as list) & return & "Anzahl Zeilen: " & anzZeilen   � ��� � d i s p l a y   d i a l o g   " Z e i l e n l i s t e :   "   &   r e t u r n   &   ( Z e i l e n l i s t e   a s   l i s t )   &   r e t u r n   &   " A n z a h l   Z e i l e n :   "   &   a n z Z e i l e n� ��� l ++����  �  �  � ��� l ++����  � � �display dialog "Zeilenliste: " & return & item 1 of Zeilenliste & return & item 2 of Zeilenliste & return & item 3 of Zeilenliste & return & item 4 of Zeilenliste & return & item 5 of Zeilenliste   � ���� d i s p l a y   d i a l o g   " Z e i l e n l i s t e :   "   &   r e t u r n   &   i t e m   1   o f   Z e i l e n l i s t e   &   r e t u r n   &   i t e m   2   o f   Z e i l e n l i s t e   &   r e t u r n   &   i t e m   3   o f   Z e i l e n l i s t e   &   r e t u r n   &   i t e m   4   o f   Z e i l e n l i s t e   &   r e t u r n   &   i t e m   5   o f   Z e i l e n l i s t e� ��� r  +;��� n  +7��� 4  .7��
� 
cobj� l 16���� \  16��� o  14�� 0 	anzzeilen 	anzZeilen� m  45�� �  �  � o  +.�� 0 zeilenliste Zeilenliste� o      �� 0 version1 Version1� ��� r  <J��� n  <F��� 4  ?F��
� 
cobj� o  BE�� 0 	anzzeilen 	anzZeilen� o  <?�� 0 zeilenliste Zeilenliste� o      �� 0 version2 Version2� ��� r  KV��� o  KN�� 0 olddels oldDels� n     ��� 1  QU�
� 
txdl� 1  NQ�

�
 
ascr� ��� l WW�	���	  �  �  � ��� l WW����  � 2 ,set main to file "datum.c" of homeordnerpfad   � ��� X s e t   m a i n   t o   f i l e   " d a t u m . c "   o f   h o m e o r d n e r p f a d� ��� r  Wd��� b  W`��� l W\���� c  W\��� o  WX�� 0 homeordnerpfad  � m  X[�
� 
TEXT�  �  � m  \_�� ���  v e r s i o n . c� o      �� 0 filepfad  � ��� l ee� ���   � , &display dialog "filepfad: " & filepfad   � ��� L d i s p l a y   d i a l o g   " f i l e p f a d :   "   &   f i l e p f a d� ��� r  e���� b  e���� b  e���� b  e�   b  e� b  e| b  ex b  et	 b  ep

 b  el m  eh �  / m  hk �  / m  lo �  v e r s i o n . c	 o  ps��
�� 
ret  m  tw �   # d e f i n e   V E R S I O N   m  x{ �  " o  |���� 0 version1 Version1 m  �� �  .� o  ������ 0 version2 Version2� m  �� �  "� o      ���� 0 
erstertext 
ersterText�  l ������   9 3display dialog "erster Text: " & return & neuerText    �   f d i s p l a y   d i a l o g   " e r s t e r   T e x t :   "   &   r e t u r n   &   n e u e r T e x t !"! l ����������  ��  ��  " #$# I ��������
�� .miscactvnull��� ��� obj ��  ��  $ %&% r  ��'(' l ��)����) I ����*+
�� .rdwropenshor       file* 4  ����,
�� 
file, o  ������ 0 filepfad  + ��-��
�� 
perm- m  ����
�� boovtrue��  ��  ��  ( o      ���� 0 refnum RefNum& ./. Q  �W0120 k  �.33 454 r  ��676 l ��8����8 I ����9��
�� .rdwrread****        ****9 o  ������ 0 refnum RefNum��  ��  ��  7 o      ���� 0 filecontents fileContents5 :;: l ����<=��  < 7 1display dialog "inhalt: " & return & fileContents   = �>> b d i s p l a y   d i a l o g   " i n h a l t :   "   &   r e t u r n   &   f i l e C o n t e n t s; ?@? l ����������  ��  ��  @ ABA r  ��CDC n  ��EFE 4  ����G
�� 
cparG m  ������ F o  ������ 0 filecontents fileContentsD o      ���� 0 alteversion  B HIH l ����JK��  J . (display dialog "Version: " & alteversion   K �LL P d i s p l a y   d i a l o g   " V e r s i o n :   "   &   a l t e v e r s i o nI MNM r  ��OPO n  ��QRQ m  ����
�� 
nmbrR n  ��STS 2 ����
�� 
cha T o  ������ 0 alteversion  P o      ���� 0 l  N UVU l ��WXYW r  ��Z[Z n  ��\]\ 7 ����^_
�� 
ctxt^ m  ������ _ m  ������ ] o  ������ 0 alteversion  [ o      ���� 0 neueversion neueVersionX $  Anfang bis und mit Leerschlag   Y �`` <   A n f a n g   b i s   u n d   m i t   L e e r s c h l a gV aba l ����������  ��  ��  b cdc r  �efe b  �ghg b  �iji b  � klk b  ��mnm b  ��opo b  ��qrq b  ��sts n  ��uvu 4  ����w
�� 
cparw m  ������ v o  ������ 0 filecontents fileContentst o  ����
�� 
ret r o  ������ 0 neueversion neueVersionp m  ��xx �yy  "n o  ������ 0 version1 Version1l m  ��zz �{{  .j o   ���� 0 version2 Version2h m  || �}}  "f o      ���� 0 	neuertext 	neuerTextd ~~ l ������  � 4 .set paragraph 2 of fileContents to neueVersion   � ��� \ s e t   p a r a g r a p h   2   o f   f i l e C o n t e n t s   t o   n e u e V e r s i o n ��� l ������  � : 4display dialog "neue Version: " & return & neuerText   � ��� h d i s p l a y   d i a l o g   " n e u e   V e r s i o n :   "   &   r e t u r n   &   n e u e r T e x t� ��� I ����
�� .rdwrseofnull���     ****� o  ���� 0 refnum RefNum� �����
�� 
set2� m  ����  ��  � ��� I &����
�� .rdwrwritnull���     ****� o  ���� 0 	neuertext 	neuerText� �����
�� 
refn� o  "���� 0 refnum RefNum��  � ���� I '.�����
�� .rdwrclosnull���     ****� o  '*���� 0 refnum RefNum��  ��  1 R      ������
�� .ascrerr ****      � ****��  ��  2 k  6W�� ��� Q  6O���� k  9@�� ��� l 99������  � w qset ersterText to "/" & "/" & "version.c" & return & "#define VERSION " & "\"" & Version1 & "." & Version2 & "\""   � ��� � s e t   e r s t e r T e x t   t o   " / "   &   " / "   &   " v e r s i o n . c "   &   r e t u r n   &   " # d e f i n e   V E R S I O N   "   &   " \ " "   &   V e r s i o n 1   &   " . "   &   V e r s i o n 2   &   " \ " "� ��� l 99������  � : 4display dialog "erstes  File: " & return & neuerText   � ��� h d i s p l a y   d i a l o g   " e r s t e s     F i l e :   "   &   r e t u r n   &   n e u e r T e x t� ���� I 9@�����
�� .rdwrclosnull���     ****� o  9<���� 0 refnum RefNum��  ��  � R      ������
�� .ascrerr ****      � ****��  ��  � I HO�����
�� .rdwrclosnull���     ****� o  HK���� 0 refnum RefNum��  � ���� I PW�����
�� .rdwrclosnull���     ****� o  PS���� 0 refnum RefNum��  ��  / ��� l XX��������  ��  ��  � ��� n  X]��� I  Y]�������� $0 logaktualisieren LogAktualisieren��  ��  �  f  XY� ��� l ^^��������  ��  ��  � ���� I ^f�����
�� .aevtodocnull  �    alis� n  ^b��� 4  _b���
�� 
file� o  `a���� $0 xcodeprojektname XcodeProjektname� o  ^_���� 0 homeordnerpfad  ��  ��   	 m     ���                                                                                  MACS  alis    r  Macintosh HD               �� �H+   �:
Finder.app                                                      Ƙh        ����  	                CoreServices    ǿ�      ƘK�     �:  ��  ��  3Macintosh HD:System:Library:CoreServices:Finder.app    
 F i n d e r . a p p    M a c i n t o s h   H D  &System/Library/CoreServices/Finder.app  / ��  ��  ��    ��� l     ��������  ��  ��  � ���� i     ��� I      �������� $0 logaktualisieren LogAktualisieren��  ��  � O    ���� k   ��� ��� I   	������
�� .miscactvnull��� ��� obj ��  ��  � ��� l  
 
��������  ��  ��  � ��� r   
 ��� m   
 �� ���  � o      ���� 0 filecontents fileContents� ��� r    ��� 4    ���
�� 
alis� l   ������ l   ������ I   ����
�� .earsffdralis        afdr�  f    � �����
�� 
rtyp� m    ��
�� 
ctxt��  ��  ��  ��  ��  � o      �� 0 
homeordner  � ��� l   �~���~  � 0 *display dialog "homeordner: " & homeordner   � ��� T d i s p l a y   d i a l o g   " h o m e o r d n e r :   "   &   h o m e o r d n e r� ��� r     ��� n    ��� m    �}
�} 
ctnr� o    �|�| 0 
homeordner  � o      �{�{ 0 homeordnerpfad  � ��� l  ! !�z���z  �  open homeordnerpfad   � ��� & o p e n   h o m e o r d n e r p f a d� ��� l  ! !�y���y  � 8 2display dialog "homeordnerpfad: " & homeordnerpfad   � ��� d d i s p l a y   d i a l o g   " h o m e o r d n e r p f a d :   "   &   h o m e o r d n e r p f a d� ��� l  ! !�x���x  � 2 ,set main to file "datum.c" of homeordnerpfad   � ��� X s e t   m a i n   t o   f i l e   " d a t u m . c "   o f   h o m e o r d n e r p f a d� ��� r   ! (��� b   ! &��� l  ! $��w�v� c   ! $��� o   ! "�u�u 0 homeordnerpfad  � m   " #�t
�t 
TEXT�w  �v  � m   $ %�� ���  L o g f i l e . t x t� o      �s�s 0 filepfad  � ��� l  ) )�r���r  � , &display dialog "filepfad: " & filepfad   � ��� L d i s p l a y   d i a l o g   " f i l e p f a d :   "   &   f i l e p f a d� ��� l  ) )�q�p�o�q  �p  �o  �    I  ) .�n�m�l
�n .miscactvnull��� ��� obj �m  �l    r   / 6 I  / 4�k�j�i
�k .misccurdldt    ��� null�j  �i   o      �h�h 	0 heute    l  7 7�g	�g   &  display dialog "heute: " & heute   	 �

 @ d i s p l a y   d i a l o g   " h e u t e :   "   &   h e u t e  r   7 < n   7 : 1   8 :�f
�f 
year o   7 8�e�e 	0 heute   o      �d�d 0 jahrtext    r   = B n   = @ m   > @�c
�c 
mnth o   = >�b�b 	0 heute   o      �a�a 0 	monattext    l  C C�`�`   * $display dialog "monat: " & monattext    � H d i s p l a y   d i a l o g   " m o n a t :   "   &   m o n a t t e x t  r   C T n   C R !  7  H R�_"#
�_ 
ctxt" m   L N�^�^��# m   O Q�]�]��! l  C H$�\�[$ b   C H%&% m   C D'' �((  0& n   D G)*) 1   E G�Z
�Z 
day * o   D E�Y�Y 	0 heute  �\  �[   o      �X�X 0 tag   +,+ l  U U�W-.�W  - " display dialog "tag: " & tag   . �// 8 d i s p l a y   d i a l o g   " t a g :   "   &   t a g, 010 r   U 232 J   U }44 565 m   U X�V
�V 
jan 6 787 m   X [�U
�U 
feb 8 9:9 m   [ ^�T
�T 
mar : ;<; l 	 ^ a=�S�R= m   ^ a�Q
�Q 
apr �S  �R  < >?> m   a d�P
�P 
may ? @A@ m   d g�O
�O 
jun A BCB m   g j�N
�N 
jul C DED m   j m�M
�M 
aug E FGF l 	 m pH�L�KH m   m p�J
�J 
sep �L  �K  G IJI m   p s�I
�I 
oct J KLK m   s v�H
�H 
nov L M�GM m   v y�F
�F 
dec �G  3 o      �E�E 0 monatsliste MonatsListe1 NON Y   � �P�DQR�CP Z   � �ST�B�AS =   � �UVU o   � ��@�@ 0 	monattext  V n   � �WXW 4   � ��?Y
�? 
cobjY o   � ��>�> 0 i  X o   � ��=�= 0 monatsliste MonatsListeT k   � �ZZ [\[ r   � �]^] n   � �_`_ 7  � ��<ab
�< 
ctxta m   � ��;�;��b m   � ��:�:��` l  � �c�9�8c b   � �ded m   � �ff �gg  0e o   � ��7�7 0 i  �9  �8  ^ o      �6�6 	0 monat  \ h�5h l  � �ijki  S   � �j - ' wenn true, wird die Schleife verlassen   k �ll N   w e n n   t r u e ,   w i r d   d i e   S c h l e i f e   v e r l a s s e n�5  �B  �A  �D 0 i  Q m   � ��4�4 R m   � ��3�3 �C  O mnm l  � ��2op�2  o &  display dialog "monat: " & monat   p �qq @ d i s p l a y   d i a l o g   " m o n a t :   "   &   m o n a tn rsr r   � �tut l 	 � �v�1�0v l  � �w�/�.w n  � �xyx 7  � ��-z{
�- 
cha z m   � ��,�, { m   � ��+�+ y l  � �|�*�)| c   � �}~} o   � ��(�( 0 jahrtext  ~ m   � ��'
�' 
ctxt�*  �)  �/  �.  �1  �0  u o      �&�& 0 jahr  s � l  � ��%���%  � ? 9display dialog "jahr: " & jahr & " jahrtext: " & jahrtext   � ��� r d i s p l a y   d i a l o g   " j a h r :   "   &   j a h r   &   "   j a h r t e x t :   "   &   j a h r t e x t� ��� l  � ��$���$  � , &set l to number of characters of Datum   � ��� L s e t   l   t o   n u m b e r   o f   c h a r a c t e r s   o f   D a t u m� ��� l  � ��#���#  � 1 +set neuesDatum to text -l thru -13 of Datum   � ��� V s e t   n e u e s D a t u m   t o   t e x t   - l   t h r u   - 1 3   o f   D a t u m� ��� l  � ��"���"  � P Jset neuesDatum to text 1 thru 14 of Datum -- Anfang bis und mit Leerschlag   � ��� � s e t   n e u e s D a t u m   t o   t e x t   1   t h r u   1 4   o f   D a t u m   - -   A n f a n g   b i s   u n d   m i t   L e e r s c h l a g� ��� r   � ���� b   � ���� b   � ���� b   � ���� b   � ���� o   � ��!�! 0 tag  � m   � ��� ���  .� o   � �� �  	0 monat  � m   � ��� ���  .� o   � ��� 0 jahrtext  � o      �� 0 
neuesdatum 
neuesDatum� ��� l  � �����  � 0 *display dialog "neuesDatum: " & neuesDatum   � ��� T d i s p l a y   d i a l o g   " n e u e s D a t u m :   "   &   n e u e s D a t u m� ��� l  � �����  �  �  � ��� l  � �����  �  �  � ��� r   � ���� l  � ����� I  � ����
� .rdwropenshor       file� 4   � ���
� 
file� o   � ��� 0 filepfad  � ���
� 
perm� m   � ��
� boovtrue�  �  �  � o      �� 0 refnum RefNum� ��� Q   �T���� k   �G�� ��� r   � ���� l  � ����� I  � ����

� .rdwrread****        ****� o   � ��	�	 0 refnum RefNum�
  �  �  � o      �� 0 filecontents fileContents� ��� r   ���� n   � ���� 4  � ���
� 
cwor� m   � ������ l  � ����� n   � ���� 4   � ���
� 
cpar� m   � ��� � o   � ��� 0 filecontents fileContents�  �  � o      � �  0 	lastdatum 	lastDatum� ��� l ������  � 7 1display dialog "lastDatum: " & return & lastDatum   � ��� b d i s p l a y   d i a l o g   " l a s t D a t u m :   "   &   r e t u r n   &   l a s t D a t u m� ��� l ������  � . (set Datum to paragraph 2 of fileContents   � ��� P s e t   D a t u m   t o   p a r a g r a p h   2   o f   f i l e C o n t e n t s� ��� l ������  � &  display dialog "Datum: " & Datum   � ��� @ d i s p l a y   d i a l o g   " D a t u m :   "   &   D a t u m� ��� Z  A������ = ��� o  ���� 0 
neuesdatum 
neuesDatum� o  ���� 0 	lastdatum 	lastDatum� l ������  � % display dialog "gleiches Datum"   � ��� > d i s p l a y   d i a l o g   " g l e i c h e s   D a t u m "��  � k  A�� ��� l ��������  ��  ��  � ��� r  -��� b  +��� b  '��� b  %��� b  !��� b  ��� b  ��� b  ��� b  ��� m  �� ��� T * * * * * * * * * * * * * * * * * * * * * *                                        � o  ���� 0 
neuesdatum 
neuesDatum� o  ��
�� 
ret � l 	������ o  ��
�� 
ret ��  ��  � o  ��
�� 
ret � l 	 ������ m      � , * * * * * * * * * * * * * * * * * * * * * *��  ��  � o  !$��
�� 
ret � o  %&���� 0 filecontents fileContents� o  '*��
�� 
ret � o      ���� 0 	neuertext 	neuerText�  I .7��
�� .rdwrseofnull���     **** o  ./���� 0 refnum RefNum ����
�� 
set2 m  23����  ��   �� I 8A��	
�� .rdwrwritnull���     **** o  89���� 0 	neuertext 	neuerText	 ��
��
�� 
refn
 o  <=���� 0 refnum RefNum��  ��  � �� I BG����
�� .rdwrclosnull���     **** o  BC���� 0 refnum RefNum��  ��  � R      ������
�� .ascrerr ****      � ****��  ��  � k  OT  l OO��������  ��  ��   �� I OT����
�� .rdwrclosnull���     **** o  OP���� 0 refnum RefNum��  ��  �  l UU����    start    � 
 s t a r t  r  U^ J  UZ �� m  UX �  x c o d e p r o j��   o      ���� 0 filetype     l __��!"��  ! ? 9set projektpfad to (path to alias (homeordner)) as string   " �## r s e t   p r o j e k t p f a d   t o   ( p a t h   t o   a l i a s   ( h o m e o r d n e r ) )   a s   s t r i n g  $%$ l __��&'��  & 0 *display dialog "projektpfad" & projektpfad   ' �(( T d i s p l a y   d i a l o g   " p r o j e k t p f a d "   &   p r o j e k t p f a d% )*) l __��+,��  + 8 2display dialog "homeordnerpfad: " & homeordnerpfad   , �-- d d i s p l a y   d i a l o g   " h o m e o r d n e r p f a d :   "   &   h o m e o r d n e r p f a d* ./. l __��01��  0 > 8get name of folders of folder (homeordnerpfad as string)   1 �22 p g e t   n a m e   o f   f o l d e r s   o f   f o l d e r   ( h o m e o r d n e r p f a d   a s   s t r i n g )/ 343 l _s5675 r  _s898 n  _o:;: 1  ko��
�� 
pnam; n  _k<=< 2 gk��
�� 
file= 4  _g��>
�� 
cfol> l cf?����? c  cf@A@ o  cd���� 0 homeordnerpfad  A m  de��
�� 
TEXT��  ��  9 o      ���� 
0 inhalt  6  without invisibles   7 �BB $ w i t h o u t   i n v i s i b l e s4 CDC l tt��EF��  E # display dialog inhalt as text   F �GG : d i s p l a y   d i a l o g   i n h a l t   a s   t e x tD HIH l tt��JK��  J 7 1repeat with i from 1 to number of items of inhalt   K �LL b r e p e a t   w i t h   i   f r o m   1   t o   n u m b e r   o f   i t e m s   o f   i n h a l tI M��M X  t�N��ON k  ��PP QRQ l ����ST��  S &  display dialog (dasFile) as text   T �UU @ d i s p l a y   d i a l o g   ( d a s F i l e )   a s   t e x tR V��V Z  ��WX����W E  ��YZY l ��[����[ l ��\����\ o  ������ 0 dasfile dasFile��  ��  ��  ��  Z m  ��]] �^^  x c o d e p r o jX k  ��__ `a` r  ��bcb b  ��ded l ��f����f c  ��ghg o  ������ 0 homeordnerpfad  h m  ����
�� 
ctxt��  ��  e l ��i����i c  ��jkj o  ������ 0 dasfile dasFilek m  ����
�� 
ctxt��  ��  c o      ���� 0 filepfad  a lml l ����no��  n &  display dialog (dasFile) as text   o �pp @ d i s p l a y   d i a l o g   ( d a s F i l e )   a s   t e x tm q��q I ����r��
�� .aevtodocnull  �    alisr 4  ����s
�� 
files o  ������ 0 filepfad  ��  ��  ��  ��  ��  �� 0 dasfile dasFileO o  wz���� 
0 inhalt  ��  � m     tt�                                                                                  MACS  alis    r  Macintosh HD               �� �H+   �:
Finder.app                                                      Ƙh        ����  	                CoreServices    ǿ�      ƘK�     �:  ��  ��  3Macintosh HD:System:Library:CoreServices:Finder.app    
 F i n d e r . a p p    M a c i n t o s h   H D  &System/Library/CoreServices/Finder.app  / ��  ��       "��uvwxyz{|}��~������������������������������  u  ����������������������������������������������������������~�}�|�� $0 logaktualisieren LogAktualisieren
�� .aevtoappnull  �   � ****�� $0 xcodeprojektname XcodeProjektname�� 0 filecontents fileContents�� 0 
homeordner  �� 0 homeordnerpfad  �� 0 dateienliste Dateienliste�� 0 filepfad  �� 0 refnum RefNum�� 0 datum Datum�� 	0 heute  �� 0 jahrtext  �� 0 	monattext  �� 0 tag  �� 0 monatsliste MonatsListe�� 	0 monat  �� 0 jahr  �� 0 l  �� 0 
neuesdatum 
neuesDatum�� 0 	neuertext 	neuerText�� 0 projektname Projektname�� 0 olddels oldDels�� 0 zeilenliste Zeilenliste�� 0 	anzzeilen 	anzZeilen�� 0 version1 Version1�� 0 version2 Version2�� 0 
erstertext 
ersterText�� 0 alteversion  � 0 neueversion neueVersion�~  �}  �|  v �{��z�y���x�{ $0 logaktualisieren LogAktualisieren�z  �y  � �w�v�u�t�s�r�q�p�o�n�m�l�k�j�i�h�g�f�e�w 0 filecontents fileContents�v 0 
homeordner  �u 0 homeordnerpfad  �t 0 filepfad  �s 	0 heute  �r 0 jahrtext  �q 0 	monattext  �p 0 tag  �o 0 monatsliste MonatsListe�n 0 i  �m 	0 monat  �l 0 jahr  �k 0 
neuesdatum 
neuesDatum�j 0 refnum RefNum�i 0 	lastdatum 	lastDatum�h 0 	neuertext 	neuerText�g 0 filetype  �f 
0 inhalt  �e 0 dasfile dasFile� :t�d��c�b�a�`�_�^��]�\�['�Z�Y�X�W�V�U�T�S�R�Q�P�O�N�M�L�Kf�J�I���H�G�F�E�D�C��B �A�@�?�>�=�<�;�:�9�8�7]�6
�d .miscactvnull��� ��� obj 
�c 
alis
�b 
rtyp
�a 
ctxt
�` .earsffdralis        afdr
�_ 
ctnr
�^ 
TEXT
�] .misccurdldt    ��� null
�\ 
year
�[ 
mnth
�Z 
day �Y��
�X 
jan 
�W 
feb 
�V 
mar 
�U 
apr 
�T 
may 
�S 
jun 
�R 
jul 
�Q 
aug 
�P 
sep 
�O 
oct 
�N 
nov 
�M 
dec �L 
�K 
cobj
�J 
cha �I 
�H 
file
�G 
perm
�F .rdwropenshor       file
�E .rdwrread****        ****
�D 
cpar
�C 
cwor
�B 
ret 
�A 
set2
�@ .rdwrseofnull���     ****
�? 
refn
�> .rdwrwritnull���     ****
�= .rdwrclosnull���     ****�<  �;  
�: 
cfol
�9 
pnam
�8 
kocl
�7 .corecnte****       ****
�6 .aevtodocnull  �    alis�x���*j O�E�O*�)��l /E�O��,E�O��&�%E�O*j O*j 
E�O��,E�O��,E�O���,%[�\[Z�\Zi2E�Oa a a a a a a a a a a a a vE�O 2ka kh 	��a �/  a �%[�\[Z�\Zi2E�OY h[OY��O��&[a \[Zm\Za  2E�O�a !%�%a "%�%E�O*a #�/a $el %E�O `�j &E�O�a 'k/a (i/E�O��  hY 7a )�%_ *%_ *%_ *%a +%_ *%�%_ *%E�O�a ,jl -O�a .�l /O�j 0W X 1 2�j 0Oa 3kvE^ O*a 4��&/a #-a 5,E^ O >] [a 6a l 7kh ] a 8 ��&] �&%E�O*a #�/j 9Y h[OY��Uw �5��4�3���2
�5 .aevtoappnull  �   � ****� k    g��  �1�1  �4  �3  � �0�/�0 0 tempname  �/ 0 i  � g� �. �-�,�+�*�)�(�'�&�%�$�#�"�!�  H� ^�������������� ��������
�	���������  �����������������FHJL�����������������������������������������������xz|�����. $0 xcodeprojektname XcodeProjektname�- 0 filecontents fileContents
�, 
alis
�+ 
rtyp
�* 
ctxt
�) .earsffdralis        afdr�( 0 
homeordner  
�' 
ctnr�& 0 homeordnerpfad  
�% 
file
�$ 
pnam�# 0 dateienliste Dateienliste
�" 
kocl
�! 
cobj
�  .corecnte****       ****
� 
TEXT� 0 filepfad  
� .miscactvnull��� ��� obj 
� 
perm
� .rdwropenshor       file� 0 refnum RefNum
� .rdwrread****        ****
� 
cpar� 0 datum Datum
� .misccurdldt    ��� null� 	0 heute  
� 
year� 0 jahrtext  
� 
mnth� 0 	monattext  
� 
day ���� 0 tag  
� 
jan 
� 
feb 
� 
mar 
�
 
apr 
�	 
may 
� 
jun 
� 
jul 
� 
aug 
� 
sep 
� 
oct 
� 
nov 
� 
dec � �  0 monatsliste MonatsListe�� 	0 monat  
�� 
cha �� �� 0 jahr  
�� 
nmbr�� 0 l  �� �� 0 
neuesdatum 
neuesDatum
�� 
ret �� 0 	neuertext 	neuerText
�� 
set2
�� .rdwrseofnull���     ****
�� 
refn
�� .rdwrwritnull���     ****
�� .rdwrclosnull���     ****��  ��  �� 0 projektname Projektname
�� 
ascr
�� 
txdl�� 0 olddels oldDels
�� 
citm�� 0 zeilenliste Zeilenliste�� 0 	anzzeilen 	anzZeilen�� 0 version1 Version1�� 0 version2 Version2�� 0 
erstertext 
ersterText�� 0 alteversion  �� �� 0 neueversion neueVersion�� $0 logaktualisieren LogAktualisieren
�� .aevtodocnull  �    alis�2h�d�E�O�E�O*�)��l /E�O��,E�O��-�,E�O '�[�a l kh  �a  
�E�OPY h[OY��O�a &a %E` O*j O*�_ /a el E` OU_ j E�O�a l/E` O*j E` O_ a ,E`  O_ a !,E` "Oa #_ a $,%[�\[Za %\Zi2E` &Oa 'a (a )a *a +a ,a -a .a /a 0a 1a 2a 3vE` 4O :ka 3kh _ "_ 4a �/  a 5�%[�\[Za %\Zi2E` 6OY h[OY��O_  �&[a 7\[Zm\Za 82E` 9O_ a 7-a :,E` ;O_ [�\[Zk\Za <2E` =O_ =a >%_ &%a ?%_ 6%a @%_  %a A%E` =O�a k/_ B%_ =%E` CO_ a Djl EO_ Ca F_ l GO_ j HW X I J_ j HOa KE�O*�)��l /E�O��,E�O��,E` LO_ Ma N,E` OOa P_ Ma N,FO_ La Q-E` RO_ Ra :,E` SO_ Ra _ Sk/E` TO_ Ra _ S/E` UO_ O_ Ma N,FO�a &a V%E` Oa Wa X%a Y%_ B%a Z%a [%_ T%a \%_ U%a ]%E` ^O*j O*�_ /a el E` O �_ j E�O�a l/E` _O_ _a 7-a :,E` ;O_ _[�\[Zk\Za `2E` aO�a k/_ B%_ a%a b%_ T%a c%_ U%a d%E` CO_ a Djl EO_ Ca F_ l GO_ j HW (X I J _ j HW X I J_ j HO_ j HO)j+ eO���/j fUx �� |����| ����� �  ������������������������� ��� @ D a t u m _ V e r s i o n _ a k t u a l i s i e r e n . s c p t� ���  L o g f i l e . t x t� ���  M a k e f i l e� ���  T W I - C h e c k� ��� ( T W I _ S l a v e   W o Z i _ 6 0 _ 8 0� ���  T W I _ S l a v e . c� ���  T W I _ S l a v e . e e p� ���  T W I _ S l a v e . e l f� ���  T W I _ S l a v e . h e x� ���  T W I _ S l a v e . l s s� ���  T W I _ S l a v e . l s t� ���  T W I _ S l a v e . m a p� ���  T W I _ S l a v e . o� ���  T W I _ S l a v e . s y m� ��� & T W I _ S l a v e . x c o d e p r o j� ���  _ _ a v r _ g d b i n i t� ��� 
 a d c . c� ��� 
 a d c . h� ���  d a t u m . c� ��� 
 l c d . c� ��� 
 l c d . h� ���  r u n a v a r i c e . s h� ���  t w i s l a v e . c� ���  v e r s i o n . c
�� 
cobj�� y ��� D / / v e r s i o n . c  # d e f i n e   V E R S I O N   " 8 0 . 1 "z �� ����� ����� ����� ����� ����� ����� ����� ����� ����� ����� ����� ���
�� 
sdsk
�� 
cfol� ��� 
 U s e r s
�� 
cfol� ���  s y s a d m i n
�� 
cfol� ���  D o c u m e n t s
�� 
cfol� ���  E l e k t r o n i k
�� 
cfol� ���    A V R
�� 
cfol� ���    A V R _ P r o g r a m m e
�� 
cfol� ���  A V R   H o m e C e n t r a l
�� 
cfol� ��� 
 S l a v e
�� 
cfol� ���  2   W o Z i
�� 
cfol� ��� , T W I _ S l a v e _ 6 6   W o Z i _ 8 0 _ 1
�� 
docf� ��� @ D a t u m _ V e r s i o n _ a k t u a l i s i e r e n . s c p t{ �� ����� ����� ����� ����� ����� ����� ����� ����� ����� ����� ���
�� 
sdsk
�� 
cfol� ��� 
 U s e r s
�� 
cfol� ���  s y s a d m i n
�� 
cfol� �    D o c u m e n t s
�� 
cfol� �  E l e k t r o n i k
�� 
cfol� �    A V R
�� 
cfol� �    A V R _ P r o g r a m m e
�� 
cfol� �  A V R   H o m e C e n t r a l
�� 
cfol� � 
 S l a v e
�� 
cfol� �  2   W o Z i
�� 
cfol� � , T W I _ S l a v e _ 6 6   W o Z i _ 8 0 _ 1} � M a c i n t o s h   H D : U s e r s : s y s a d m i n : D o c u m e n t s : E l e k t r o n i k :   A V R :   A V R _ P r o g r a m m e : A V R   H o m e C e n t r a l : S l a v e : 2   W o Z i : T W I _ S l a v e _ 6 6   W o Z i _ 8 0 _ 1 : v e r s i o n . c��~ �		 4 # d e f i n e   D A T U M   " 1 8 . 1 2 . 2 0 1 0 " ldt     �2Fz���
�� 
dec � �

  1 8� ����   ������������������������
�� 
jan 
�� 
feb 
�� 
mar 
�� 
apr 
�� 
may 
�� 
jun 
�� 
jul 
�� 
aug 
�� 
sep 
�� 
oct 
�� 
nov 
�� 
dec � �  1 2� ����   ���������������������������� �  1 �  0��  ��  ��  ��  ��  ��  ��  ��  ��  ��  ��  ��  ��  ��  �� � � 4 # d e f i n e   D A T U M   " 1 8 . 1 2 . 2 0 1 0 "� � D / / v e r s i o n . c  # d e f i n e   V E R S I O N   " 8 0 . 1 "� � , T W I _ S l a v e _ 6 6   W o Z i _ 8 0 _ 1� ����    �  � ����   ������������������������ �  T W I � 
 S l a v e �  6 6   W o Z i� �  8 0� �    1��  ��  ��  ��  ��  ��  ��  ��  ��  ��  ��  �� � �!! D / / v e r s i o n . c  # d e f i n e   V E R S I O N   " 8 0 . 1 "� �"" , # d e f i n e   V E R S I O N   " 8 0 . 1 "� �##   # d e f i n e   V E R S I O N  ��  ��  ��   ascr  ��ޭ