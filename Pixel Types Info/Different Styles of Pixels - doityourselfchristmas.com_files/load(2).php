var mediaWikiLoadStart=(new Date()).getTime(),mwPerformance=(window.performance&&performance.mark)?performance:{mark:function(){}};mwPerformance.mark('mwLoadStart');function isCompatible(str){var ua=str||navigator.userAgent;return!!('querySelector'in document&&'localStorage'in window&&'addEventListener'in window&&!(ua.match(/webOS\/1\.[0-4]/)||ua.match(/PlayStation/i)||ua.match(/SymbianOS|Series60|NetFront|Opera Mini|S40OviBrowser|MeeGo/)||(ua.match(/Glass/)&&ua.match(/Android/))));}(function(){var NORLQ,script;if(!isCompatible()){document.documentElement.className=document.documentElement.className.replace(/(^|\s)client-js(\s|$)/,'$1client-nojs$2');NORLQ=window.NORLQ||[];while(NORLQ.length){NORLQ.shift()();}window.NORLQ={push:function(fn){fn();}};window.RLQ={push:function(){}};return;}function startUp(){mw.config=new mw.Map(true);mw.loader.addSource({"local":"/wiki/load.php"});mw.loader.register([["site","cChHG0cL"],["noscript","D+AY6507",[],"noscript"],["filepage","ZpvMdQNq"],[
"user.groups","aMwbh3cC",[],"user"],["user","bsuW2wAc",[],"user"],["user.cssprefs","GqV9IPpY",[],"private"],["user.defaults","pPg/TJeS"],["user.options","C9rS/VRT",[6],"private"],["user.tokens","pQ3B6z1y",[],"private"],["mediawiki.language.data","bHbxm6Jh",[174]],["mediawiki.skinning.elements","JLD33B2k"],["mediawiki.skinning.content","gnBFSyVg"],["mediawiki.skinning.interface","VvpjUpka"],["mediawiki.skinning.content.parsoid","UNKW/HhM"],["mediawiki.skinning.content.externallinks","fLNsrBi7"],["jquery.accessKeyLabel","OtkybqDQ",[25,130]],["jquery.appear","SbvjTScY"],["jquery.arrowSteps","qb+GY0eg"],["jquery.async","LbZ6wFz3"],["jquery.autoEllipsis","XPoTX58g",[37]],["jquery.badge","22R0BE/o",[171]],["jquery.byteLength","r0S/2wH7"],["jquery.byteLimit","I2jG0nG0",[21]],["jquery.checkboxShiftClick","1gbSyYUI"],["jquery.chosen","VnaV1D8S"],["jquery.client","KqhqgAlC"],["jquery.color","3OJSfH60",[27]],["jquery.colorUtil","kBM11qXN"],["jquery.confirmable","fM/X0KrD",[175]],["jquery.cookie",
"KBJ7WZtA"],["jquery.expandableField","Fi+ZxJxi"],["jquery.farbtastic","XHBR4U8t",[27]],["jquery.footHovzer","9yb4A4TI"],["jquery.form","F6IipCpj"],["jquery.fullscreen","ICIcK4vQ"],["jquery.getAttrs","QnA9Ghtm"],["jquery.hidpi","ulmwO4kR"],["jquery.highlightText","t7ayhuHO",[242,130]],["jquery.hoverIntent","iuK/w62S"],["jquery.i18n","ge4d7zoi",[173]],["jquery.localize","+r+t9B/p"],["jquery.makeCollapsible","ifEIqsdN"],["jquery.mockjax","AvY0XyIj"],["jquery.mw-jump","4bLSz9W6"],["jquery.mwExtension","czur9c52"],["jquery.placeholder","J9MjLKlt"],["jquery.qunit","4mTo1ZL4"],["jquery.qunit.completenessTest","NdoWONfs",[46]],["jquery.spinner","oGxbPLAJ"],["jquery.jStorage","7h+QAvt0",[92]],["jquery.suggestions","x5aL/4jp",[37]],["jquery.tabIndex","P9mtkTnJ"],["jquery.tablesorter","i2luqFrS",[242,130,176]],["jquery.textSelection","vbffFw1V",[25]],["jquery.throttle-debounce","9jKAHLTY"],["jquery.xmldom","Etsrtwd4"],["jquery.tipsy","w2vppec/"],["jquery.ui.core","FbR43pt/",[58],"jquery.ui"],[
"jquery.ui.core.styles","OMk4F8n2",[],"jquery.ui"],["jquery.ui.accordion","Fv7BZm30",[57,77],"jquery.ui"],["jquery.ui.autocomplete","2P87M6DC",[66],"jquery.ui"],["jquery.ui.button","T6GZf57o",[57,77],"jquery.ui"],["jquery.ui.datepicker","MxKtxbSz",[57],"jquery.ui"],["jquery.ui.dialog","BPHiHuuF",[61,64,68,70],"jquery.ui"],["jquery.ui.draggable","eV8pnnOO",[57,67],"jquery.ui"],["jquery.ui.droppable","EZnUdjqh",[64],"jquery.ui"],["jquery.ui.menu","F9dW0tXv",[57,68,77],"jquery.ui"],["jquery.ui.mouse","LqR3eZEU",[77],"jquery.ui"],["jquery.ui.position","cFfrltip",[],"jquery.ui"],["jquery.ui.progressbar","Jjwh5xFU",[57,77],"jquery.ui"],["jquery.ui.resizable","diqAqewC",[57,67],"jquery.ui"],["jquery.ui.selectable","2lwJCtEc",[57,67],"jquery.ui"],["jquery.ui.slider","EoYRRRS/",[57,67],"jquery.ui"],["jquery.ui.sortable","ad2coBiP",[57,67],"jquery.ui"],["jquery.ui.spinner","GySndHs7",[61],"jquery.ui"],["jquery.ui.tabs","QgsQkjtK",[57,77],"jquery.ui"],["jquery.ui.tooltip","sCdchTj8",[57,68,77],
"jquery.ui"],["jquery.ui.widget","4fFUs8Ad",[],"jquery.ui"],["jquery.effects.core","3gHLco67",[],"jquery.ui"],["jquery.effects.blind","OaWmM2S7",[78],"jquery.ui"],["jquery.effects.bounce","pPSmKURN",[78],"jquery.ui"],["jquery.effects.clip","7KWn+8bZ",[78],"jquery.ui"],["jquery.effects.drop","Uw32lOpX",[78],"jquery.ui"],["jquery.effects.explode","66/TOxeL",[78],"jquery.ui"],["jquery.effects.fade","XC3VDfke",[78],"jquery.ui"],["jquery.effects.fold","wKQKBklS",[78],"jquery.ui"],["jquery.effects.highlight","tCF/7tMP",[78],"jquery.ui"],["jquery.effects.pulsate","Qscd8Td8",[78],"jquery.ui"],["jquery.effects.scale","dRy1IYzk",[78],"jquery.ui"],["jquery.effects.shake","CXShkCk+",[78],"jquery.ui"],["jquery.effects.slide","uJqjnz0j",[78],"jquery.ui"],["jquery.effects.transfer","pKGOHVLD",[78],"jquery.ui"],["json","56k3viZc",[],null,null,"return!!(window.JSON\u0026\u0026JSON.stringify\u0026\u0026JSON.parse);"],["moment","3g9fxUMI"],["mediawiki.apihelp","yEmA3dpP"],["mediawiki.template","r7lS+1UB"
],["mediawiki.template.mustache","K4W4oPOr",[95]],["mediawiki.template.regexp","up4JWcVR",[95]],["mediawiki.apipretty","KRWSDdhL"],["mediawiki.api","stlJIcYy",[147,8]],["mediawiki.api.category","eyXOZZv6",[135,99]],["mediawiki.api.edit","nmSpImwa",[135,99]],["mediawiki.api.login","wZ7ZIsgT",[99]],["mediawiki.api.options","lRFf5qGv",[99]],["mediawiki.api.parse","451SgIq5",[99]],["mediawiki.api.upload","BT6DTpUH",[242,92,101]],["mediawiki.api.user","OA9PDOSh",[99]],["mediawiki.api.watch","m0Bb4pj/",[99]],["mediawiki.api.messages","LhqO+Q12",[99]],["mediawiki.content.json","9O1mNB8h"],["mediawiki.confirmCloseWindow","WnzVqBm2"],["mediawiki.debug","6i53UeLr",[32,56]],["mediawiki.debug.init","X2MGB3No",[111]],["mediawiki.feedback","viodgC6j",[135,126,250]],["mediawiki.feedlink","IGjL1pXi"],["mediawiki.filewarning","hadLZXBz",[245]],["mediawiki.ForeignApi","L5w1MZis",[117]],["mediawiki.ForeignApi.core","GNWdmGko",[99,243]],["mediawiki.helplink","naKlJMJa"],["mediawiki.hidpi","2n20mWDi",[36],
null,null,"return'srcset'in new Image();"],["mediawiki.hlist","6WnepKsv"],["mediawiki.htmlform","niCx79FN",[22,130]],["mediawiki.htmlform.styles","97Eyxy/c"],["mediawiki.htmlform.ooui.styles","rKYf2ymP"],["mediawiki.icon","3jBuoNrk"],["mediawiki.inspect","LEU562mU",[21,92,130]],["mediawiki.messagePoster","M4cOdds+",[116]],["mediawiki.messagePoster.wikitext","kmjKwdLm",[101,126]],["mediawiki.notification","lxLsn+gW",[183]],["mediawiki.notify","FWUG/D8e"],["mediawiki.RegExp","Qii45jz1"],["mediawiki.pager.tablePager","gdLrM2H3"],["mediawiki.searchSuggest","N9lewbmJ",[35,45,50,99]],["mediawiki.sectionAnchor","KL2dR6Qf"],["mediawiki.storage","3mR9onmz"],["mediawiki.Title","Ckf93/S2",[21,147]],["mediawiki.Upload","zRQ0hZzO",[105]],["mediawiki.ForeignUpload","nXgqXBdY",[116,136]],["mediawiki.ForeignStructuredUpload.config","JtmRymYi"],["mediawiki.ForeignStructuredUpload","2ySbC3gz",[138,137]],["mediawiki.Upload.Dialog","dn8ewdS0",[141]],["mediawiki.Upload.BookletLayout","OHHPC/GB",[136,175,
145,240,93,250,256,257]],["mediawiki.ForeignStructuredUpload.BookletLayout","+JSFxb0i",[139,141,108,179,236,234]],["mediawiki.toc","TMXaky4V",[151]],["mediawiki.Uri","1p6vfJ6v",[147,97]],["mediawiki.user","GN+5/AwR",[106,151,7]],["mediawiki.userSuggest","waKc1DSk",[50,99]],["mediawiki.util","UYaqbj9d",[15,129]],["mediawiki.viewport","G48vrk9Q"],["mediawiki.checkboxtoggle","lSchW1v2"],["mediawiki.checkboxtoggle.styles","rVbgcOHa"],["mediawiki.cookie","Td/JTTDK",[29]],["mediawiki.toolbar","qaDT5B3w"],["mediawiki.experiments","l7cbEyDL"],["mediawiki.raggett","UJpL+EQQ"],["mediawiki.action.edit","PLZQVkaq",[22,53,156]],["mediawiki.action.edit.styles","Ue3qZUSl"],["mediawiki.action.edit.collapsibleFooter","PnL7dlxk",[41,151,124]],["mediawiki.action.edit.preview","CPearB4K",[33,48,53,161,99,175]],["mediawiki.action.edit.stash","Xm3QpL/s",[35,99]],["mediawiki.action.history","IRxShvlB"],["mediawiki.action.history.diff","0eFCf0SW"],["mediawiki.action.view.dblClickEdit","/2ZZl6RK",[183,7]],[
"mediawiki.action.view.metadata","/eC3onYp"],["mediawiki.action.view.categoryPage.styles","NAnsPB/V"],["mediawiki.action.view.postEdit","3voHEfjc",[151,175,95]],["mediawiki.action.view.redirect","bodxrGY5",[25]],["mediawiki.action.view.redirectPage","8tZkDb9X"],["mediawiki.action.view.rightClickEdit","SNE8V0cq"],["mediawiki.action.edit.editWarning","IkL2Jfqp",[53,110,175]],["mediawiki.action.view.filepage","fAW0Dr5o"],["mediawiki.language","VCn6hUnX",[172,9]],["mediawiki.cldr","hgxcZyX3",[173]],["mediawiki.libs.pluralruleparser","HSB9yP3o"],["mediawiki.language.init","lx414oCm"],["mediawiki.jqueryMsg","3BFJOerX",[242,171,147,7]],["mediawiki.language.months","gJ/uBjZA",[171]],["mediawiki.language.names","dVn8oVHH",[174]],["mediawiki.language.specialCharacters","v31HqVfX",[171]],["mediawiki.libs.jpegmeta","dE5tAo5B"],["mediawiki.page.gallery","PqwHMOaC",[54,181]],["mediawiki.page.gallery.styles","bcyyhK1r"],["mediawiki.page.ready","tRc0kKIk",[15,23,41,43,45]],["mediawiki.page.startup",
"8k5Doblv",[147]],["mediawiki.page.patrol.ajax","gX5L06xJ",[48,135,99,183]],["mediawiki.page.watch.ajax","Y3R/MOpx",[107,183]],["mediawiki.page.image.pagination","9QWsIlJR",[48,147]],["mediawiki.special","CvgqqWXb"],["mediawiki.special.apisandbox.styles","Z9FU8/Mm"],["mediawiki.special.apisandbox","BGcCNit2",[99,175,187,235,244]],["mediawiki.special.block","L+stT85q",[147]],["mediawiki.special.blocklist","2Fg8LLyV"],["mediawiki.special.changeslist","GcMeVJCK"],["mediawiki.special.changeslist.legend","rgDimML7"],["mediawiki.special.changeslist.legend.js","SlH/RW2r",[41,151]],["mediawiki.special.changeslist.enhanced","sKX2zI4Y"],["mediawiki.special.changeslist.visitedstatus","mKUzeC1u"],["mediawiki.special.comparepages.styles","T5XHFUjE"],["mediawiki.special.edittags","rJNqvgu8",[24]],["mediawiki.special.edittags.styles","yodvaeqb"],["mediawiki.special.import","r09ujLa+"],["mediawiki.special.movePage","eccC8SSS",[232]],["mediawiki.special.movePage.styles","Tq2t7MaI"],[
"mediawiki.special.pageLanguage","5nMithMH",[245]],["mediawiki.special.pagesWithProp","uySv3YF+"],["mediawiki.special.preferences","cGFlvpwJ",[110,171,128]],["mediawiki.special.preferences.styles","xstdsI13"],["mediawiki.special.recentchanges","/jPzRg2T",[187]],["mediawiki.special.search","KvVVOqfs",[238]],["mediawiki.special.undelete","lKk5JilV"],["mediawiki.special.upload","BZQPNhN/",[48,135,99,110,175,179,95]],["mediawiki.special.userlogin.common.styles","96LFP9j7"],["mediawiki.special.userlogin.signup.styles","DCyY0Z9F"],["mediawiki.special.userlogin.login.styles","PuZRJ9fa"],["mediawiki.special.userlogin.signup.js","Mi+sWtKI",[54,99,175]],["mediawiki.special.unwatchedPages","+2CsSAmq",[135,107]],["mediawiki.special.watchlist","+3sy4rUL"],["mediawiki.special.version","kyOqnfh8"],["mediawiki.legacy.config","o+lIagh7"],["mediawiki.legacy.commonPrint","XKgnHv7Z"],["mediawiki.legacy.protect","hDjzwZY5",[22]],["mediawiki.legacy.shared","2UVfgA1B"],["mediawiki.legacy.oldshared",
"LJePZ0v0"],["mediawiki.legacy.wikibits","oorlNw+j",[147]],["mediawiki.ui","9ffxSYZ/"],["mediawiki.ui.checkbox","hGqBfDfG"],["mediawiki.ui.radio","ft3RVAJg"],["mediawiki.ui.anchor","GQil01I8"],["mediawiki.ui.button","Yy1/8Ar6"],["mediawiki.ui.input","TwefHZma"],["mediawiki.ui.icon","7YBUsNdK"],["mediawiki.ui.text","xImhb+0E"],["mediawiki.widgets","veadrRdq",[19,22,135,99,233,248]],["mediawiki.widgets.styles","lZhy9pqH"],["mediawiki.widgets.DateInputWidget","vO1Mp7y0",[93,248]],["mediawiki.widgets.datetime","lJCklQnI",[245]],["mediawiki.widgets.CategorySelector","OjFH2B0T",[116,135,248]],["mediawiki.widgets.UserInputWidget","EGwUooVS",[248]],["mediawiki.widgets.SearchInputWidget","4matFb+i",[132,232]],["mediawiki.widgets.SearchInputWidget.styles","s4RL4dJF"],["mediawiki.widgets.StashedFileWidget","YkxbYrEG",[245]],["es5-shim","gfCtNd3f",[],null,null,"return(function(){'use strict';return!this\u0026\u0026!!Function.prototype.bind;}());"],["dom-level2-shim","hLEmcCIt",[],null,null,
"return!!window.Node;"],["oojs","Muik7krf",[241,92]],["oojs-ui","hPfmf2pt",[249,248,250]],["oojs-ui-core","EfRKVItB",[171,243,246]],["oojs-ui-core.styles","fFuSYyCD",[251,252,253],null,null,"return!!jQuery('meta[name=\"X-OOUI-PHP\"]').length;"],["oojs-ui.styles","FZl+4Vnx",[251,252,253],null,null,"return!!jQuery('meta[name=\"X-OOUI-PHP\"]').length;"],["oojs-ui-widgets","g34NyOp4",[245]],["oojs-ui-toolbars","aB21CWpW",[245]],["oojs-ui-windows","3ELRNlDK",[245]],["oojs-ui.styles.icons","e70VwvUc"],["oojs-ui.styles.indicators","qvyZdIbW"],["oojs-ui.styles.textures","QLXXQuKi"],["oojs-ui.styles.icons-accessibility","dUCscr95"],["oojs-ui.styles.icons-alerts","7+gFDgsW"],["oojs-ui.styles.icons-content","0/jHJxqj"],["oojs-ui.styles.icons-editing-advanced","wVmbqGZe"],["oojs-ui.styles.icons-editing-core","NZ6uLX8F"],["oojs-ui.styles.icons-editing-list","ScZEuArE"],["oojs-ui.styles.icons-editing-styling","PtRP4j87"],["oojs-ui.styles.icons-interactions","/h8+E9qD"],["oojs-ui.styles.icons-layout"
,"ncFSOEx1"],["oojs-ui.styles.icons-location","kX0iqxsR"],["oojs-ui.styles.icons-media","emfBLh3h"],["oojs-ui.styles.icons-moderation","ffpglKwU"],["oojs-ui.styles.icons-movement","C9ifweb5"],["oojs-ui.styles.icons-user","+9GofU2y"],["oojs-ui.styles.icons-wikimedia","z10xEoZ+"],["skins.cologneblue","zD8o8Lms"],["skins.modern","LUwQ0NXb"],["skins.monobook.styles","h5g/42it"],["skins.vector.styles","vKloYKAf"],["skins.vector.styles.responsive","RlthDqL7"],["skins.vector.js","KfXhHlGo",[51,54]]]);;mw.config.set({"wgLoadScript":"/wiki/load.php","debug":!1,"skin":"monobook","stylepath":"/wiki/skins","wgUrlProtocols":"bitcoin\\:|ftp\\:\\/\\/|ftps\\:\\/\\/|geo\\:|git\\:\\/\\/|gopher\\:\\/\\/|http\\:\\/\\/|https\\:\\/\\/|irc\\:\\/\\/|ircs\\:\\/\\/|magnet\\:|mailto\\:|mms\\:\\/\\/|news\\:|nntp\\:\\/\\/|redis\\:\\/\\/|sftp\\:\\/\\/|sip\\:|sips\\:|sms\\:|ssh\\:\\/\\/|svn\\:\\/\\/|tel\\:|telnet\\:\\/\\/|urn\\:|worldwind\\:\\/\\/|xmpp\\:|\\/\\/","wgArticlePath":"/wiki/index.php?title=$1",
"wgScriptPath":"/wiki","wgScriptExtension":".php","wgScript":"/wiki/index.php","wgSearchType":null,"wgVariantArticlePath":!1,"wgActionPaths":{},"wgServer":"https://www.doityourselfchristmas.com","wgServerName":"www.doityourselfchristmas.com","wgUserLanguage":"en","wgContentLanguage":"en","wgTranslateNumerals":!0,"wgVersion":"1.27.1","wgEnableAPI":!0,"wgEnableWriteAPI":!0,"wgMainPageTitle":"Main Page","wgFormattedNamespaces":{"-2":"Media","-1":"Special","0":"","1":"Talk","2":"User","3":"User talk","4":"doityourselfchristmas.com","5":"doityourselfchristmas.com talk","6":"File","7":"File talk","8":"MediaWiki","9":"MediaWiki talk","10":"Template","11":"Template talk","12":"Help","13":"Help talk","14":"Category","15":"Category talk"},"wgNamespaceIds":{"media":-2,"special":-1,"":0,"talk":1,"user":2,"user_talk":3,"doityourselfchristmas.com":4,"doityourselfchristmas.com_talk":5,"file":6,"file_talk":7,"mediawiki":8,"mediawiki_talk":9,"template":10,"template_talk":11,"help":12,
"help_talk":13,"category":14,"category_talk":15,"image":6,"image_talk":7,"project":4,"project_talk":5},"wgContentNamespaces":[0],"wgSiteName":"doityourselfchristmas.com","wgDBname":"doityour_wikidb","wgExtraSignatureNamespaces":[],"wgAvailableSkins":{"cologneblue":"CologneBlue","modern":"Modern","monobook":"MonoBook","vector":"Vector","fallback":"Fallback","apioutput":"ApiOutput"},"wgExtensionAssetsPath":"/wiki/extensions","wgCookiePrefix":"doityour_wikidb","wgCookieDomain":"","wgCookiePath":"/","wgCookieExpiration":15552000,"wgResourceLoaderMaxQueryLength":2000,"wgCaseSensitiveNamespaces":[],"wgLegalTitleChars":" %!\"$&'()*,\\-./0-9:;=?@A-Z\\\\\\^_`a-z~+\\u0080-\\uFFFF","wgResourceLoaderStorageVersion":1,"wgResourceLoaderStorageEnabled":!1,"wgResourceLoaderLegacyModules":[],"wgForeignUploadTargets":[],"wgEnableUploads":!0});var RLQ=window.RLQ||[];while(RLQ.length){RLQ.shift()();}window.RLQ={push:function(fn){fn();}};window.NORLQ={push:function(){}};}script=document.createElement(
'script');script.src="/wiki/load.php?debug=false&lang=en&modules=jquery%2Cmediawiki&only=scripts&skin=monobook&version=fe%2FnOgAS";script.onload=script.onreadystatechange=function(){if(!script.readyState||/loaded|complete/.test(script.readyState)){script.onload=script.onreadystatechange=null;script=null;startUp();}};document.getElementsByTagName('head')[0].appendChild(script);}());