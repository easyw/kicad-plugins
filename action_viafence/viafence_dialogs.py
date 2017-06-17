# -*- coding: utf-8 -*- 

###########################################################################
## Python code generated with wxFormBuilder (version Feb 16 2016)
## http://www.wxformbuilder.org/
##
## PLEASE DO "NOT" EDIT THIS FILE!
###########################################################################

import wx
import wx.xrc

###########################################################################
## Class MainDialogBase
###########################################################################

class MainDialogBase ( wx.Dialog ):
	
	def __init__( self, parent ):
		wx.Dialog.__init__ ( self, parent, id = wx.ID_ANY, title = u"Via Fence Generator", pos = wx.DefaultPosition, size = wx.Size( -1,-1 ), style = wx.CAPTION|wx.CLOSE_BOX )
		
		self.SetSizeHintsSz( wx.Size( -1,-1 ), wx.Size( 400,600 ) )
		
		mainSizer = wx.BoxSizer( wx.VERTICAL )
		
		gbSizer4 = wx.GridBagSizer( 0, 0 )
		gbSizer4.SetFlexibleDirection( wx.BOTH )
		gbSizer4.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		bSizer23 = wx.BoxSizer( wx.VERTICAL )
		
		sbSizer2 = wx.StaticBoxSizer( wx.StaticBox( self, wx.ID_ANY, u"Via Settings" ), wx.VERTICAL )
		
		fgSizer4 = wx.FlexGridSizer( 8, 2, 0, 10 )
		fgSizer4.AddGrowableCol( 1 )
		fgSizer4.SetFlexibleDirection( wx.BOTH )
		fgSizer4.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_staticText11 = wx.StaticText( sbSizer2.GetStaticBox(), wx.ID_ANY, u"Offset (mm):", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText11.Wrap( -1 )
		fgSizer4.Add( self.m_staticText11, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 5 )
		
		self.txtViaOffset = wx.TextCtrl( sbSizer2.GetStaticBox(), wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_RIGHT )
		fgSizer4.Add( self.txtViaOffset, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.m_staticText21 = wx.StaticText( sbSizer2.GetStaticBox(), wx.ID_ANY, u"Pitch (mm):", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText21.Wrap( -1 )
		fgSizer4.Add( self.m_staticText21, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 5 )
		
		self.txtViaPitch = wx.TextCtrl( sbSizer2.GetStaticBox(), wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_RIGHT )
		fgSizer4.Add( self.txtViaPitch, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.m_staticText13 = wx.StaticText( sbSizer2.GetStaticBox(), wx.ID_ANY, u"Via Drill (mm):", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText13.Wrap( -1 )
		fgSizer4.Add( self.m_staticText13, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 5 )
		
		self.txtViaDrill = wx.TextCtrl( sbSizer2.GetStaticBox(), wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_RIGHT )
		fgSizer4.Add( self.txtViaDrill, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.m_staticText14 = wx.StaticText( sbSizer2.GetStaticBox(), wx.ID_ANY, u"Via Size (mm):", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText14.Wrap( -1 )
		fgSizer4.Add( self.m_staticText14, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 5 )
		
		self.txtViaSize = wx.TextCtrl( sbSizer2.GetStaticBox(), wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_RIGHT )
		fgSizer4.Add( self.txtViaSize, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.m_staticText23 = wx.StaticText( sbSizer2.GetStaticBox(), wx.ID_ANY, u"Via Net:", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText23.Wrap( -1 )
		fgSizer4.Add( self.m_staticText23, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 5 )
		
		self.txtViaNet = wx.TextCtrl( sbSizer2.GetStaticBox(), wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer4.Add( self.txtViaNet, 0, wx.ALL|wx.EXPAND, 5 )
		
		
		sbSizer2.Add( fgSizer4, 1, wx.EXPAND, 5 )
		
		
		bSizer23.Add( sbSizer2, 1, wx.ALL|wx.EXPAND, 5 )
		
		
		gbSizer4.Add( bSizer23, wx.GBPosition( 0, 0 ), wx.GBSpan( 1, 1 ), wx.EXPAND, 5 )
		
		bSizer21 = wx.BoxSizer( wx.VERTICAL )
		
		self.bmpViafence = wx.StaticBitmap( self, wx.ID_ANY, wx.NullBitmap, wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer21.Add( self.bmpViafence, 1, wx.ALL|wx.EXPAND, 5 )
		
		
		gbSizer4.Add( bSizer21, wx.GBPosition( 0, 1 ), wx.GBSpan( 1, 1 ), wx.EXPAND|wx.ALL, 5 )
		
		sbSizer411 = wx.StaticBoxSizer( wx.StaticBox( self, wx.ID_ANY, u"Input Tracks" ), wx.VERTICAL )
		
		gSizer4 = wx.GridSizer( 3, 1, 0, 0 )
		
		self.chkIncludeDrawing = wx.CheckBox( sbSizer411.GetStaticBox(), wx.ID_ANY, u"Include Drawing Lines", wx.DefaultPosition, wx.DefaultSize, 0 )
		gSizer4.Add( self.chkIncludeDrawing, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL|wx.EXPAND, 5 )
		
		fgSizer311 = wx.FlexGridSizer( 1, 2, 0, 10 )
		fgSizer311.AddGrowableCol( 1 )
		fgSizer311.SetFlexibleDirection( wx.HORIZONTAL )
		fgSizer311.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.chkNetFilter = wx.CheckBox( sbSizer411.GetStaticBox(), wx.ID_ANY, u"Net(s):", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer311.Add( self.chkNetFilter, 0, wx.ALL|wx.EXPAND, 5 )
		
		txtNetFilterChoices = []
		self.txtNetFilter = wx.ComboBox( sbSizer411.GetStaticBox(), wx.ID_ANY, u"Combo!", wx.DefaultPosition, wx.DefaultSize, txtNetFilterChoices, 0 )
		self.txtNetFilter.Enable( False )
		
		fgSizer311.Add( self.txtNetFilter, 1, wx.ALL|wx.EXPAND, 5 )
		
		
		gSizer4.Add( fgSizer311, 1, wx.EXPAND|wx.ALIGN_CENTER_VERTICAL, 5 )
		
		fgSizer31 = wx.FlexGridSizer( 1, 2, 0, 10 )
		fgSizer31.AddGrowableCol( 1 )
		fgSizer31.SetFlexibleDirection( wx.HORIZONTAL )
		fgSizer31.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.chkLayer = wx.CheckBox( sbSizer411.GetStaticBox(), wx.ID_ANY, u"Layer:", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer31.Add( self.chkLayer, 1, wx.ALL|wx.EXPAND, 5 )
		
		lstLayerChoices = []
		self.lstLayer = wx.Choice( sbSizer411.GetStaticBox(), wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, lstLayerChoices, 0 )
		self.lstLayer.SetSelection( 0 )
		self.lstLayer.Enable( False )
		
		fgSizer31.Add( self.lstLayer, 1, wx.ALL|wx.EXPAND, 5 )
		
		
		gSizer4.Add( fgSizer31, 1, wx.EXPAND|wx.ALIGN_CENTER_VERTICAL, 5 )
		
		
		sbSizer411.Add( gSizer4, 0, wx.EXPAND, 5 )
		
		
		gbSizer4.Add( sbSizer411, wx.GBPosition( 1, 0 ), wx.GBSpan( 1, 1 ), wx.ALL|wx.EXPAND, 5 )
		
		
		gbSizer4.AddGrowableCol( 0 )
		
		mainSizer.Add( gbSizer4, 1, wx.EXPAND, 5 )
		
		bSizer5 = wx.BoxSizer( wx.HORIZONTAL )
		
		self.btnSave = wx.Button( self, wx.ID_ANY, u"Save...", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.btnSave.Enable( False )
		
		bSizer5.Add( self.btnSave, 0, wx.ALL, 5 )
		
		
		bSizer5.AddSpacer( ( 0, 0), 1, wx.EXPAND, 5 )
		
		m_sdbSizer1 = wx.StdDialogButtonSizer()
		self.m_sdbSizer1OK = wx.Button( self, wx.ID_OK )
		m_sdbSizer1.AddButton( self.m_sdbSizer1OK )
		self.m_sdbSizer1Cancel = wx.Button( self, wx.ID_CANCEL )
		m_sdbSizer1.AddButton( self.m_sdbSizer1Cancel )
		m_sdbSizer1.Realize();
		
		bSizer5.Add( m_sdbSizer1, 0, wx.RIGHT|wx.EXPAND, 5 )
		
		
		mainSizer.Add( bSizer5, 0, wx.EXPAND, 5 )
		
		
		self.SetSizer( mainSizer )
		self.Layout()
		mainSizer.Fit( self )
		
		self.Centre( wx.BOTH )
		
		# Connect Events
		self.chkNetFilter.Bind( wx.EVT_CHECKBOX, self.OnNetFilterCheckBox )
		self.chkLayer.Bind( wx.EVT_CHECKBOX, self.OnLayerCheckBox )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def OnNetFilterCheckBox( self, event ):
		event.Skip()
	
	def OnLayerCheckBox( self, event ):
		event.Skip()
	

