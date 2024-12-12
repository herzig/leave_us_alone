using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UIElements;



public class UIScript : MonoBehaviour
{
    private Toggle interactorToggle;
    private Toggle animatorToggle;
    private Button homeAButton;
    private Button homeBButton;
    private Button gotoBaseButton;

    private Interactor interactor;
    private GameObject cams;

    //Add logic that interacts with the UI controls in the `OnEnable` methods
    private void OnEnable()
    {
        // The UXML is already instantiated by the UIDocument component
        var uiDocument = GetComponent<UIDocument>();

        interactorToggle = uiDocument.rootVisualElement.Q("interactorToggle") as Toggle;
        animatorToggle = uiDocument.rootVisualElement.Q("animatorToggle") as Toggle;
        homeAButton = uiDocument.rootVisualElement.Q("homeAButton") as Button;
        homeBButton = uiDocument.rootVisualElement.Q("homeBButton") as Button;
        gotoBaseButton = uiDocument.rootVisualElement.Q("gotoBaseButton") as Button;

        cams = GameObject.Find("cams");
        interactor = GameObject.Find("Interactor").GetComponent<Interactor>();

        interactorToggle.value = interactor.gameObject.activeInHierarchy;

        interactorToggle.RegisterCallback<ChangeEvent<bool>>(InteractorToggleChanged);

        animatorToggle.value = cams.GetComponent<Animator>().enabled;
        animatorToggle.RegisterCallback<ChangeEvent<bool>>(AnimatorToggleChanged);

        homeAButton.RegisterCallback<ClickEvent>(HomeAClick);
        homeBButton.RegisterCallback<ClickEvent>(HomeBClick);
        gotoBaseButton.RegisterCallback<ClickEvent>(gotoBaseButtonClick);

    }

    private void OnDisable()
    {
        interactorToggle.UnregisterCallback<ChangeEvent<bool>>(InteractorToggleChanged);
    }

    private void InteractorToggleChanged(ChangeEvent<bool> e)
    {
        interactor.gameObject.SetActive(interactorToggle.value);
    }
    private void AnimatorToggleChanged(ChangeEvent<bool> e)
    {
        var anim = cams.GetComponent<Animator>();

        if (animatorToggle.value)
        {
            cams.GetComponent<TwoRobots>().StartAnimation();
        }
        else
        {
            cams.GetComponent<TwoRobots>().PauseAnimation();
        }
        anim.enabled = animatorToggle.value;
    }

    private void HomeAClick(ClickEvent e)
    {
        var twoBots = cams.GetComponent<TwoRobots>();
        twoBots.camA.Home();
    }
    private void HomeBClick(ClickEvent e)
    {
        var twoBots = cams.GetComponent<TwoRobots>();
        twoBots.camB.Home();
    }

    private void gotoBaseButtonClick(ClickEvent e)
    {
        var twoBots = cams.GetComponent<TwoRobots>();
        twoBots.aPan = twoBots.camA.lastAnimatedPanTilt[0];
        twoBots.aTilt = twoBots.camA.lastAnimatedPanTilt[1];

        twoBots.bPan = twoBots.camB.lastAnimatedPanTilt[0];
        twoBots.bTilt = twoBots.camB.lastAnimatedPanTilt[1];

        twoBots.MoveUDP();
    }

}
