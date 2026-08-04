			// More info about initialization & config:
			// - https://revealjs.com/initialization/
			// - https://revealjs.com/config/
			Reveal.initialize({
				pdfMaxPagesPerSlide: 1,
				width: 1333,
  				height: 750,
				center:false,
				katex: {
    version: 'latest',
    delimiters: [
      {left: '$$', right: '$$', display: true},
      {left: '$', right: '$', display: false},
      {left: '\\(', right: '\\)', display: false},
      {left: '\\[', right: '\\]', display: true}
   ],
   ignoredTags: ['script', 'noscript', 'style', 'textarea', 'pre'],
   macros: {
	"\\R": "\\mathbb{R}",
	"\\trp": "\\intercal",
	"\\diag": "\\text{diag}",
	"\\Diag": "\\text{Diag}",
	"\\tr": "\\text{tr}",
	"\\stairs": "\\star",
	"\\Fscr": "\\mathcal{F}",
	"\\Bscr": "\\mathcal{B}",
	"\\Prob": "\\mathbb{P}",
	"\\Om": "\\Omega",
	"\\om": "\\omega",
	"\\E":"\\mathbb{E}",
	"\\dif": "d",
	"\\Var":"\\text{Var}",
	"\\Cov":"\\text{Cov}",
	"\\Corr":"\\text{Corr}"
   }
 },
				hash: true,

				// highlight: {
				// 	beforeHighlight: (hljs) => hljs.registerLanguage('javascript', require('highlight.js/lib/languages/javascript')),
				//   },
				plugins: [ RevealMarkdown, RevealHighlight, RevealNotes, RevealMath.KaTeX ],
				controls: true,
				controlsLayout: 'edges',
				progress: false,
				transition: 'none'
			});
		//////remove slide number from first page (data-hide-slide-number="true")
	    Reveal.addEventListener('slidechanged', (event) => {
  		const isSnOn = (event.currentSlide.dataset.hideSlideNumber !== 'true');
  		Reveal.configure({ slideNumber: isSnOn });
		});
	